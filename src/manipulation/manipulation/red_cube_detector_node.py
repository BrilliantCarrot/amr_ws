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


class RedCubeDetectorNode(Node):
    """Detect a red object in RGB-D and report its 3D camera-frame point."""

    def __init__(self):
        super().__init__("red_cube_detector_node")

        self.declare_parameter("rgb_topic", "/station_camera/image_raw")
        self.declare_parameter("depth_topic", "/station_camera/depth")
        self.declare_parameter("camera_info_topic", "/station_camera/camera_info")
        self.declare_parameter("min_red", 90)
        self.declare_parameter("red_margin", 35)
        self.declare_parameter("min_area_px", 20)
        self.declare_parameter("depth_window_px", 5)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("publish_period_sec", 0.20)
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("transform_mode", "manual")
        self.declare_parameter("camera_frame_fallback", "station_camera_rgb")
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("enable_manual_panda_transform", False)
        self.declare_parameter("camera_to_panda_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("camera_to_panda_quaternion", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("grasp_offset_xyz", [0.0, 0.0, 0.117])
        self.declare_parameter("pick_orientation_xyzw", [1.0, 0.0, 0.0, 0.0])

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.min_red = int(self.get_parameter("min_red").value)
        self.red_margin = int(self.get_parameter("red_margin").value)
        self.min_area_px = int(self.get_parameter("min_area_px").value)
        self.depth_window_px = int(self.get_parameter("depth_window_px").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.planning_frame = self.get_parameter("planning_frame").value
        self.transform_mode = self.get_parameter("transform_mode").value
        self.camera_frame_fallback = self.get_parameter("camera_frame_fallback").value
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)

        self.latest_rgb: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.camera_info: Optional[CameraInfo] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.rgb_sub = self.create_subscription(
            Image, self.rgb_topic, self._on_rgb, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._on_depth, qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, 10
        )
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.camera_point_pub = self.create_publisher(PointStamped, "~/point_camera", 10)
        self.pick_pose_pub = self.create_publisher(PoseStamped, "~/pick_pose_candidate", 10)
        self.debug_image_pub = self.create_publisher(Image, "~/debug_image", 10)

        period = float(self.get_parameter("publish_period_sec").value)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "Red cube detector ready: "
            f"rgb={self.rgb_topic}, depth={self.depth_topic}, info={self.camera_info_topic}, "
            f"transform_mode={self.transform_mode}, planning_frame={self.planning_frame}"
        )

    def _on_rgb(self, msg: Image):
        self.latest_rgb = msg

    def _on_depth(self, msg: Image):
        self.latest_depth = msg

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _on_timer(self):
        if self.latest_rgb is None or self.latest_depth is None or self.camera_info is None:
            self._publish_status("WAITING_FOR_RGB_DEPTH_CAMERA_INFO")
            return

        rgb = self._image_to_rgb_array(self.latest_rgb)
        depth = self._depth_to_float_array(self.latest_depth)
        if rgb is None or depth is None:
            return

        detection = self._detect_red_centroid(rgb)
        if detection is None:
            self._publish_status("NO_RED_OBJECT")
            return

        u, v, area, bbox = detection
        depth_m = self._depth_at(depth, u, v)
        if depth_m is None:
            self._publish_status(f"NO_VALID_DEPTH u={u} v={v} area={area}")
            return

        camera_point = self._project_pixel_to_3d(u, v, depth_m)
        if camera_point is None:
            self._publish_status("INVALID_CAMERA_INFO")
            return

        stamp = self.latest_depth.header.stamp
        frame_id = self._camera_frame_id()
        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = frame_id
        point_msg.point.x = camera_point[0]
        point_msg.point.y = camera_point[1]
        point_msg.point.z = camera_point[2]
        self.camera_point_pub.publish(point_msg)
        self._publish_debug_image(rgb, self.latest_rgb.header, u, v, bbox)

        status = (
            f"RED_FOUND u={u} v={v} area={area} depth={depth_m:.3f}m "
            f"bbox=[{bbox[0]}, {bbox[1]}, {bbox[2]}, {bbox[3]}] "
            f"camera_xyz=[{camera_point[0]:.3f}, {camera_point[1]:.3f}, {camera_point[2]:.3f}]"
        )

        pick_pose = self._make_pick_pose(camera_point, frame_id, stamp)
        if pick_pose is not None:
            self.pick_pose_pub.publish(pick_pose)
            p = pick_pose.pose.position
            status += f" pick_xyz=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]"

        self._publish_status(status)
        self.get_logger().info(status, throttle_duration_sec=1.0)

    def _image_to_rgb_array(self, msg: Image) -> Optional[np.ndarray]:
        channels_by_encoding = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
        }
        encoding = msg.encoding.lower()
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            self._publish_status(f"UNSUPPORTED_RGB_ENCODING {msg.encoding}")
            return None

        data = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.width * channels
        if data.size < expected:
            self._publish_status("RGB_DATA_TOO_SHORT")
            return None

        image = data[:expected].reshape((msg.height, msg.width, channels))
        if encoding in ("bgr8", "bgra8"):
            image = image[..., [2, 1, 0] + ([3] if channels == 4 else [])]
        return image[..., :3]

    def _depth_to_float_array(self, msg: Image) -> Optional[np.ndarray]:
        if msg.encoding != "32FC1":
            self._publish_status(f"UNSUPPORTED_DEPTH_ENCODING {msg.encoding}")
            return None

        data = np.frombuffer(msg.data, dtype=np.float32)
        expected = msg.height * msg.width
        if data.size < expected:
            self._publish_status("DEPTH_DATA_TOO_SHORT")
            return None
        return data[:expected].reshape((msg.height, msg.width))

    def _detect_red_centroid(self, rgb: np.ndarray) -> Optional[Tuple[int, int, int, Tuple[int, int, int, int]]]:
        r = rgb[..., 0].astype(np.int16)
        g = rgb[..., 1].astype(np.int16)
        b = rgb[..., 2].astype(np.int16)
        mask = (r >= self.min_red) & (r >= g + self.red_margin) & (r >= b + self.red_margin)
        component = self._largest_connected_component(mask)
        if component is None:
            return None

        ys, xs = np.nonzero(component)
        area = int(xs.size)
        if area < self.min_area_px:
            return None

        bbox = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
        return int(round(float(xs.mean()))), int(round(float(ys.mean()))), area, bbox

    def _largest_connected_component(self, mask: np.ndarray) -> Optional[np.ndarray]:
        height, width = mask.shape
        visited = np.zeros(mask.shape, dtype=bool)
        best_pixels = []

        starts_y, starts_x = np.nonzero(mask)
        for start_x, start_y in zip(starts_x.tolist(), starts_y.tolist()):
            if visited[start_y, start_x]:
                continue

            stack = [(start_x, start_y)]
            visited[start_y, start_x] = True
            pixels = []

            while stack:
                x, y = stack.pop()
                pixels.append((x, y))

                for ny in range(max(0, y - 1), min(height, y + 2)):
                    for nx in range(max(0, x - 1), min(width, x + 2)):
                        if visited[ny, nx] or not mask[ny, nx]:
                            continue
                        visited[ny, nx] = True
                        stack.append((nx, ny))

            if len(pixels) > len(best_pixels):
                best_pixels = pixels

        if not best_pixels:
            return None

        component = np.zeros(mask.shape, dtype=bool)
        xs, ys = zip(*best_pixels)
        component[np.array(ys), np.array(xs)] = True
        return component

    def _publish_debug_image(self, rgb: np.ndarray, header, u: int, v: int, bbox):
        debug = np.array(rgb, copy=True)
        x0, y0, x1, y1 = bbox
        height, width = debug.shape[:2]
        x0 = max(0, min(width - 1, x0))
        x1 = max(0, min(width - 1, x1))
        y0 = max(0, min(height - 1, y0))
        y1 = max(0, min(height - 1, y1))

        debug[y0 : y1 + 1, x0 : min(width, x0 + 2)] = [0, 255, 0]
        debug[y0 : y1 + 1, max(0, x1 - 1) : x1 + 1] = [0, 255, 0]
        debug[y0 : min(height, y0 + 2), x0 : x1 + 1] = [0, 255, 0]
        debug[max(0, y1 - 1) : y1 + 1, x0 : x1 + 1] = [0, 255, 0]

        cross = 5
        debug[max(0, v - cross) : min(height, v + cross + 1), u : min(width, u + 1)] = [255, 255, 0]
        debug[v : min(height, v + 1), max(0, u - cross) : min(width, u + cross + 1)] = [255, 255, 0]

        msg = Image()
        msg.header = header
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = debug.astype(np.uint8).tobytes()
        self.debug_image_pub.publish(msg)

    def _depth_at(self, depth: np.ndarray, u: int, v: int) -> Optional[float]:
        half = max(0, self.depth_window_px // 2)
        y0 = max(0, v - half)
        y1 = min(depth.shape[0], v + half + 1)
        x0 = max(0, u - half)
        x1 = min(depth.shape[1], u + half + 1)
        window = depth[y0:y1, x0:x1]
        valid = window[np.isfinite(window) & (window > 0.0) & (window < self.max_depth_m)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _project_pixel_to_3d(self, u: int, v: int, z: float) -> Optional[Tuple[float, float, float]]:
        k = self.camera_info.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])
        if fx == 0.0 or fy == 0.0:
            return None

        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        return x, y, z

    def _camera_frame_id(self) -> str:
        if self.latest_depth is not None and self.latest_depth.header.frame_id:
            return self.latest_depth.header.frame_id
        if self.latest_rgb is not None and self.latest_rgb.header.frame_id:
            return self.latest_rgb.header.frame_id
        return self.camera_frame_fallback

    def _make_pick_pose(
        self,
        camera_point: Tuple[float, float, float],
        camera_frame: str,
        stamp,
    ) -> Optional[PoseStamped]:
        mode = str(self.get_parameter("transform_mode").value).lower()
        if mode == "tf":
            pick_xyz = self._pick_xyz_from_tf(camera_point, camera_frame, stamp)
        elif mode == "manual":
            if not bool(self.get_parameter("enable_manual_panda_transform").value):
                return None
            pick_xyz = self._pick_xyz_from_manual_transform(camera_point)
        else:
            self._publish_status(f"UNSUPPORTED_TRANSFORM_MODE {mode}")
            return None

        if pick_xyz is None:
            return None

        orientation = list(self.get_parameter("pick_orientation_xyzw").value)
        if len(orientation) != 4:
            self._publish_status("INVALID_PICK_ORIENTATION")
            return None

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = float(pick_xyz[0])
        pose.pose.position.y = float(pick_xyz[1])
        pose.pose.position.z = float(pick_xyz[2])
        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])
        return pose

    def _pick_xyz_from_manual_transform(
        self,
        camera_point: Tuple[float, float, float],
    ) -> Optional[Tuple[float, float, float]]:
        translation = list(self.get_parameter("camera_to_panda_translation").value)
        quaternion = list(self.get_parameter("camera_to_panda_quaternion").value)
        offset = list(self.get_parameter("grasp_offset_xyz").value)
        if len(translation) != 3 or len(quaternion) != 4 or len(offset) != 3:
            self._publish_status("INVALID_MANUAL_TRANSFORM_PARAMS")
            return None

        rotated = self._rotate_vector_by_quaternion(camera_point, quaternion)
        return (
            rotated[0] + translation[0] + offset[0],
            rotated[1] + translation[1] + offset[1],
            rotated[2] + translation[2] + offset[2],
        )

    def _pick_xyz_from_tf(
        self,
        camera_point: Tuple[float, float, float],
        camera_frame: str,
        stamp,
    ) -> Optional[Tuple[float, float, float]]:
        transform = self._lookup_camera_to_planning_transform(camera_frame, stamp)
        if transform is None:
            return None

        target_point = self._transform_point(camera_point, transform)
        offset = list(self.get_parameter("grasp_offset_xyz").value)
        if len(offset) != 3:
            self._publish_status("INVALID_GRASP_OFFSET")
            return None

        return (
            target_point[0] + offset[0],
            target_point[1] + offset[1],
            target_point[2] + offset[2],
        )

    def _lookup_camera_to_planning_transform(self, camera_frame: str, stamp):
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

        # Quaternion-vector multiplication optimized as v' = v + 2w(q x v) + 2(q x (q x v)).
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        rx = vx + qw * tx + (qy * tz - qz * ty)
        ry = vy + qw * ty + (qz * tx - qx * tz)
        rz = vz + qw * tz + (qx * ty - qy * tx)
        return [rx, ry, rz]

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RedCubeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
