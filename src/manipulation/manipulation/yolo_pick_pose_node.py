#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray


class YoloPickPoseNode(Node):
    """Convert YOLO 2D detections plus depth into a MoveIt pick PoseStamped."""

    def __init__(self):
        super().__init__("yolo_pick_pose_node")

        self.declare_parameter("detections_topic", "/detection/objects")
        self.declare_parameter("depth_topic", "/station_camera/depth")
        self.declare_parameter("camera_info_topic", "/station_camera/camera_info")
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("camera_frame_fallback", "station_camera_rgb")
        self.declare_parameter("use_latest_tf", True)

        self.declare_parameter("target_class", "")
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("selection_strategy", "highest_confidence")

        self.declare_parameter("min_depth", 0.05)
        self.declare_parameter("max_depth", 5.0)
        self.declare_parameter("bbox_shrink_ratio", 0.10)
        self.declare_parameter("bbox_sample_stride", 2)
        self.declare_parameter("foreground_percentile", 25.0)
        self.declare_parameter("foreground_band_margin", 0.20)
        self.declare_parameter("min_valid_points", 20)
        self.declare_parameter("min_selected_points", 10)

        self.declare_parameter("grasp_offset_xyz", [0.0, 0.0, 0.117])
        self.declare_parameter("pick_orientation_xyzw", [1.0, 0.0, 0.0, 0.0])

        self.detections_topic = self.get_parameter("detections_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.planning_frame = self.get_parameter("planning_frame").value
        self.camera_frame_fallback = self.get_parameter("camera_frame_fallback").value
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)

        self.target_class = self.get_parameter("target_class").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.selection_strategy = self.get_parameter("selection_strategy").value
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.bbox_shrink_ratio = float(self.get_parameter("bbox_shrink_ratio").value)
        self.bbox_sample_stride = max(1, int(self.get_parameter("bbox_sample_stride").value))
        self.foreground_percentile = float(self.get_parameter("foreground_percentile").value)
        self.foreground_band_margin = float(self.get_parameter("foreground_band_margin").value)
        self.min_valid_points = int(self.get_parameter("min_valid_points").value)
        self.min_selected_points = int(self.get_parameter("min_selected_points").value)

        self.latest_depth: Optional[np.ndarray] = None
        self.latest_depth_encoding = ""
        self.latest_depth_frame = ""
        self.camera_info: Optional[CameraInfo] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._on_depth, qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, self.detections_topic, self._on_detections, 10
        )

        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.camera_point_pub = self.create_publisher(PointStamped, "~/point_camera", 10)
        self.pick_pose_pub = self.create_publisher(PoseStamped, "~/pick_pose", 10)

        self.get_logger().info(
            "YOLO pick pose node ready: "
            f"detections={self.detections_topic}, depth={self.depth_topic}, "
            f"camera_info={self.camera_info_topic}, target_class='{self.target_class}'"
        )

    def _on_depth(self, msg: Image):
        depth = self._depth_to_float_array(msg)
        if depth is None:
            return
        self.latest_depth = depth
        self.latest_depth_encoding = msg.encoding
        self.latest_depth_frame = msg.header.frame_id

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _on_detections(self, msg: Detection2DArray):
        if self.latest_depth is None or self.camera_info is None:
            self._publish_status("WAITING_FOR_DEPTH_CAMERA_INFO")
            return

        detection = self._select_detection(msg)
        if detection is None:
            self._publish_status("NO_USABLE_YOLO_DETECTION")
            return

        camera_point = self._project_detection(detection)
        if camera_point is None:
            return

        camera_frame = msg.header.frame_id or self.latest_depth_frame or self.camera_frame_fallback
        stamp = msg.header.stamp

        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = camera_frame
        point_msg.point.x = camera_point[0]
        point_msg.point.y = camera_point[1]
        point_msg.point.z = camera_point[2]
        self.camera_point_pub.publish(point_msg)

        pick_pose = self._make_pick_pose(camera_point, camera_frame, stamp)
        if pick_pose is None:
            return
        self.pick_pose_pub.publish(pick_pose)

        label, score = self._label_score(detection)
        p = pick_pose.pose.position
        self._publish_status(
            f"YOLO_PICK_POSE class={label} score={score:.3f} "
            f"camera_xyz=[{camera_point[0]:.3f}, {camera_point[1]:.3f}, {camera_point[2]:.3f}] "
            f"pick_xyz=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]"
        )

    def _select_detection(self, msg: Detection2DArray) -> Optional[Detection2D]:
        candidates = []
        for det in msg.detections:
            label, score = self._label_score(det)
            if score < self.min_confidence:
                continue
            if self.target_class and label != self.target_class:
                continue
            candidates.append(det)

        if not candidates:
            return None

        if self.selection_strategy == "largest_area":
            return max(candidates, key=lambda det: float(det.bbox.size_x * det.bbox.size_y))
        if self.selection_strategy == "nearest":
            scored = []
            for det in candidates:
                point = self._project_detection(det, publish_errors=False)
                if point is not None:
                    scored.append((point[2], det))
            if scored:
                return min(scored, key=lambda item: item[0])[1]

        return max(candidates, key=lambda det: self._label_score(det)[1])

    @staticmethod
    def _label_score(det: Detection2D) -> Tuple[str, float]:
        if not det.results:
            return "", 0.0
        hyp = det.results[0].hypothesis
        return hyp.class_id, float(hyp.score)

    def _project_detection(
        self,
        det: Detection2D,
        publish_errors: bool = True,
    ) -> Optional[Tuple[float, float, float]]:
        k = self.camera_info.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])
        if fx == 0.0 or fy == 0.0:
            if publish_errors:
                self._publish_status("INVALID_CAMERA_INFO")
            return None

        depth = self.latest_depth
        height, width = depth.shape[:2]

        u_center = float(det.bbox.center.position.x)
        v_center = float(det.bbox.center.position.y)
        bbox_w = float(det.bbox.size_x)
        bbox_h = float(det.bbox.size_y)

        x0 = u_center - 0.5 * bbox_w
        x1 = u_center + 0.5 * bbox_w
        y0 = v_center - 0.5 * bbox_h
        y1 = v_center + 0.5 * bbox_h

        shrink_x = self.bbox_shrink_ratio * bbox_w
        shrink_y = self.bbox_shrink_ratio * bbox_h
        x0 += shrink_x
        x1 -= shrink_x
        y0 += shrink_y
        y1 -= shrink_y

        x0_i = max(0, int(np.floor(x0)))
        x1_i = min(width - 1, int(np.ceil(x1)))
        y0_i = max(0, int(np.floor(y0)))
        y1_i = min(height - 1, int(np.ceil(y1)))
        if x1_i <= x0_i or y1_i <= y0_i:
            if publish_errors:
                self._publish_status("INVALID_BBOX")
            return None

        stride = self.bbox_sample_stride
        depth_crop = depth[y0_i : y1_i + 1 : stride, x0_i : x1_i + 1 : stride]
        v_coords, u_coords = np.mgrid[y0_i : y1_i + 1 : stride, x0_i : x1_i + 1 : stride]
        u_coords = u_coords.astype(np.float32)
        v_coords = v_coords.astype(np.float32)

        valid = np.isfinite(depth_crop)
        valid &= depth_crop > self.min_depth
        valid &= depth_crop < self.max_depth
        if int(np.count_nonzero(valid)) < self.min_valid_points:
            if publish_errors:
                self._publish_status("NO_VALID_DEPTH_IN_YOLO_BBOX")
            return None

        valid_depth = depth_crop[valid]
        foreground_depth = float(np.percentile(valid_depth, self.foreground_percentile))
        band_min = max(self.min_depth, foreground_depth - self.foreground_band_margin)
        band_max = min(self.max_depth, foreground_depth + self.foreground_band_margin)
        selected = valid & (depth_crop >= band_min) & (depth_crop <= band_max)
        if int(np.count_nonzero(selected)) < self.min_selected_points:
            if publish_errors:
                self._publish_status("NO_FOREGROUND_DEPTH_IN_YOLO_BBOX")
            return None

        selected_depth = depth_crop[selected]
        selected_u = u_coords[selected]
        selected_v = v_coords[selected]

        x_points = (selected_u - cx) * selected_depth / fx
        y_points = (selected_v - cy) * selected_depth / fy
        z_points = selected_depth
        return (
            float(np.median(x_points)),
            float(np.median(y_points)),
            float(np.median(z_points)),
        )

    def _make_pick_pose(
        self,
        camera_point: Tuple[float, float, float],
        camera_frame: str,
        stamp,
    ) -> Optional[PoseStamped]:
        transform = self._lookup_transform(camera_frame, stamp)
        if transform is None:
            return None

        target_point = self._transform_point(camera_point, transform)
        offset = list(self.get_parameter("grasp_offset_xyz").value)
        orientation = list(self.get_parameter("pick_orientation_xyzw").value)
        if len(offset) != 3 or len(orientation) != 4:
            self._publish_status("INVALID_GRASP_ORIENTATION_PARAMS")
            return None

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = target_point[0] + float(offset[0])
        pose.pose.position.y = target_point[1] + float(offset[1])
        pose.pose.position.z = target_point[2] + float(offset[2])
        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])
        return pose

    def _lookup_transform(self, camera_frame: str, stamp):
        if camera_frame == self.planning_frame:
            return self._identity_transform()
        try:
            lookup_time = Time() if self.use_latest_tf else Time.from_msg(stamp)
            return self.tf_buffer.lookup_transform(
                self.planning_frame,
                camera_frame,
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException as exc:
            self._publish_status(f"TF_LOOKUP_FAILED {self.planning_frame}<-{camera_frame}: {exc}")
            self.get_logger().warn(
                f"TF lookup failed: {self.planning_frame} <- {camera_frame}: {exc}",
                throttle_duration_sec=1.0,
            )
            return None

    @staticmethod
    def _identity_transform():
        class Translation:
            x = 0.0
            y = 0.0
            z = 0.0

        class Rotation:
            x = 0.0
            y = 0.0
            z = 0.0
            w = 1.0

        class Transform:
            translation = Translation()
            rotation = Rotation()

        class TransformStamped:
            transform = Transform()

        return TransformStamped()

    def _transform_point(self, point: Tuple[float, float, float], transform_stamped):
        t = transform_stamped.transform.translation
        q = transform_stamped.transform.rotation
        rotated = self._rotate_vector_by_quaternion(point, [q.x, q.y, q.z, q.w])
        return (
            rotated[0] + float(t.x),
            rotated[1] + float(t.y),
            rotated[2] + float(t.z),
        )

    @staticmethod
    def _rotate_vector_by_quaternion(vector, quaternion):
        qx, qy, qz, qw = [float(x) for x in quaternion]
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            return list(vector)
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        vx, vy, vz = [float(x) for x in vector]
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        rx = vx + qw * tx + (qy * tz - qz * ty)
        ry = vy + qw * ty + (qz * tx - qx * tz)
        rz = vz + qw * tz + (qx * ty - qy * tx)
        return [rx, ry, rz]

    def _depth_to_float_array(self, msg: Image) -> Optional[np.ndarray]:
        if msg.encoding == "32FC1":
            data = np.frombuffer(msg.data, dtype=np.float32)
            expected = msg.height * msg.width
            if data.size < expected:
                self._publish_status("DEPTH_DATA_TOO_SHORT")
                return None
            return data[:expected].reshape((msg.height, msg.width))

        if msg.encoding == "16UC1":
            data = np.frombuffer(msg.data, dtype=np.uint16)
            expected = msg.height * msg.width
            if data.size < expected:
                self._publish_status("DEPTH_DATA_TOO_SHORT")
                return None
            return data[:expected].reshape((msg.height, msg.width)).astype(np.float32) / 1000.0

        self._publish_status(f"UNSUPPORTED_DEPTH_ENCODING {msg.encoding}")
        return None

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloPickPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
