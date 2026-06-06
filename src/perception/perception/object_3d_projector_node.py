#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    Detection3DArray,
    Detection3D,
    ObjectHypothesisWithPose,
)
from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class DepthDebugInfo:
    # 디버깅 로그용 정보만 모아두는 구조체임
    bbox_center_u: float
    bbox_center_v: float
    bbox_width: float
    bbox_height: float
    center_depth: Optional[float]
    roi_median_depth: Optional[float]
    foreground_depth: Optional[float]
    selected_depth_median: Optional[float]
    selected_depth_min: Optional[float]
    selected_depth_max: Optional[float]
    selected_depth_std: Optional[float]
    bbox_valid_count: int
    bbox_total_count: int
    selected_count: int
    band_min: Optional[float]
    band_max: Optional[float]
    valid_ratio: float
    selected_ratio: float


@dataclass
class ProjectedObject:
    # bbox + depth crop으로 추정한 객체의 카메라 optical frame 기준 3D 위치임
    x: float
    y: float
    z: float
    debug: DepthDebugInfo


@dataclass
class TransformedObject:
    # camera frame과 target frame 위치를 함께 보관함
    camera_x: float
    camera_y: float
    camera_z: float
    target_x: Optional[float]
    target_y: Optional[float]
    target_z: Optional[float]
    target_frame: str


class Object3DProjectorNode(Node):
    def __init__(self):
        super().__init__("object_3d_projector_node")

        # -------------------------
        # Topic / frame parameters
        # -------------------------
        self.declare_parameter("detections_topic", "/detection/objects")
        self.declare_parameter("depth_topic", "/camera/depth")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

        # 카메라 좌표계 기준 Detection3DArray 출력 토픽임
        self.declare_parameter("output_camera_topic", "/detection/objects_3d")

        # base_link 같은 로봇 기준 Detection3DArray 출력 토픽임
        self.declare_parameter("output_base_topic", "/detection/objects_3d_base")

        # object 위치를 변환할 목표 frame임. 주행/회피에서는 보통 base_link를 씀
        self.declare_parameter("target_frame", "base_link")

        # Detection2DArray/header.frame_id를 우선 사용하되, 비어 있거나 강제 지정하고 싶을 때 쓰는 fallback임
        self.declare_parameter("camera_frame_fallback", "camera_rgb_optical_frame")

        # TF lookup에서 latest transform을 쓸지 결정함
        # Isaac Sim sensor timestamp와 static TF timestamp가 섞일 수 있어 우선 latest가 디버깅에 안정적임
        self.declare_parameter("use_latest_tf", True)

        # -------------------------
        # Depth estimation parameters
        # -------------------------
        self.declare_parameter("min_depth", 0.05)
        self.declare_parameter("max_depth", 20.0)

        # bbox 중심 주변 depth 디버깅용 ROI 반경임
        # 실제 최종 위치 추정에는 bbox 내부 foreground depth를 사용함
        self.declare_parameter("center_roi_radius", 3)

        # bbox 내부 전체 픽셀을 다 쓰면 연산량이 커질 수 있으므로 stride로 샘플링함
        self.declare_parameter("bbox_sample_stride", 2)

        # bbox 내부 valid depth 중 몇 percentile을 foreground 후보 깊이로 볼지 정함
        # depth는 작을수록 카메라에 가까운 표면이므로 20~30 percentile이 배경보다 객체 표면을 잡기 좋음
        self.declare_parameter("foreground_percentile", 25.0)

        # foreground depth 주변 몇 m 범위를 같은 객체 표면 후보로 볼지 정함
        self.declare_parameter("foreground_band_margin", 0.35)

        # bbox 가장자리는 배경/바닥이 많이 섞일 수 있어 조금 잘라낼 수 있음
        self.declare_parameter("bbox_shrink_ratio", 0.0)

        # bbox 안에 valid depth가 너무 적으면 잘못된 추정으로 보고 skip함
        self.declare_parameter("min_valid_points", 20)

        # foreground band에 들어온 점이 너무 적으면 skip함
        self.declare_parameter("min_selected_points", 10)

        # 로그 출력 주기 제한용임
        self.declare_parameter("log_period", 1.0)

        self.detections_topic = self.get_parameter("detections_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.output_camera_topic = self.get_parameter("output_camera_topic").value
        self.output_base_topic = self.get_parameter("output_base_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.camera_frame_fallback = self.get_parameter("camera_frame_fallback").value
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)

        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.center_roi_radius = int(self.get_parameter("center_roi_radius").value)
        self.bbox_sample_stride = max(1, int(self.get_parameter("bbox_sample_stride").value))
        self.foreground_percentile = float(self.get_parameter("foreground_percentile").value)
        self.foreground_band_margin = float(self.get_parameter("foreground_band_margin").value)
        self.bbox_shrink_ratio = float(self.get_parameter("bbox_shrink_ratio").value)
        self.min_valid_points = int(self.get_parameter("min_valid_points").value)
        self.min_selected_points = int(self.get_parameter("min_selected_points").value)
        self.log_period = float(self.get_parameter("log_period").value)

        self.bridge = CvBridge()

        # 최신 depth image와 camera_info를 캐싱해두고 detection callback에서 사용함
        self.latest_depth = None
        self.latest_depth_encoding = None
        self.latest_depth_frame_id = ""
        self.latest_depth_stamp = None
        self.latest_camera_info = None

        self.last_log_time = 0.0

        # TF buffer/listener임
        # camera_rgb_optical_frame 기준 3D 점을 base_link 기준으로 바꿀 때 사용함
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Isaac Sim sensor topic은 BEST_EFFORT QoS인 경우가 많음
        self.sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            self.sensor_qos,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            self.sensor_qos,
        )

        self.detections_sub = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detections_callback,
            10,
        )

        # camera frame 기준 3D detection 발행함
        self.objects_3d_camera_pub = self.create_publisher(
            Detection3DArray,
            self.output_camera_topic,
            10,
        )

        # target frame(base_link 등) 기준 3D detection 발행함
        self.objects_3d_base_pub = self.create_publisher(
            Detection3DArray,
            self.output_base_topic,
            10,
        )

        self.get_logger().info("Object 3D projector node started")
        self.get_logger().info(f"Subscribe detections : {self.detections_topic}")
        self.get_logger().info(f"Subscribe depth      : {self.depth_topic}")
        self.get_logger().info(f"Subscribe camera info: {self.camera_info_topic}")
        self.get_logger().info(f"Publish camera frame : {self.output_camera_topic}")
        self.get_logger().info(f"Publish target frame : {self.output_base_topic}")
        self.get_logger().info(f"Target frame         : {self.target_frame}")
        self.get_logger().info(
            "Depth strategy       : bbox foreground percentile "
            f"p={self.foreground_percentile:.1f}, band=±{self.foreground_band_margin:.2f}m, "
            f"stride={self.bbox_sample_stride}"
        )

    def depth_callback(self, msg: Image):
        try:
            # depth image는 encoding이 32FC1, 16UC1 등일 수 있으므로 passthrough로 받음
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            self.latest_depth = np.array(depth_image)
            self.latest_depth_encoding = msg.encoding
            self.latest_depth_frame_id = msg.header.frame_id
            self.latest_depth_stamp = msg.header.stamp

        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg

    def detections_callback(self, msg: Detection2DArray):
        if self.latest_depth is None:
            self.get_logger().warn("Depth image not received yet")
            return

        if self.latest_camera_info is None:
            self.get_logger().warn("CameraInfo not received yet")
            return

        fx, fy, cx, cy = self.get_camera_intrinsics()
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn("Invalid camera intrinsic fx/fy")
            return

        camera_frame = self.get_camera_frame(msg)

        out_camera_msg = Detection3DArray()
        out_camera_msg.header = Header()
        out_camera_msg.header.stamp = msg.header.stamp
        out_camera_msg.header.frame_id = camera_frame

        out_base_msg = Detection3DArray()
        out_base_msg.header = Header()
        out_base_msg.header.stamp = msg.header.stamp
        out_base_msg.header.frame_id = self.target_frame

        transform = self.lookup_camera_to_target_transform(camera_frame, msg.header.stamp)

        debug_entries = []

        for det2d in msg.detections:
            if len(det2d.results) == 0:
                continue

            projected = self.project_detection(det2d, fx, fy, cx, cy)
            if projected is None:
                continue

            det3d_camera = self.make_detection3d(det2d, out_camera_msg.header, projected.x, projected.y, projected.z)
            out_camera_msg.detections.append(det3d_camera)

            transformed = TransformedObject(
                camera_x=projected.x,
                camera_y=projected.y,
                camera_z=projected.z,
                target_x=None,
                target_y=None,
                target_z=None,
                target_frame=self.target_frame,
            )

            if transform is not None:
                target_x, target_y, target_z = self.transform_point(projected.x, projected.y, projected.z, transform)
                transformed.target_x = target_x
                transformed.target_y = target_y
                transformed.target_z = target_z

                det3d_base = self.make_detection3d(det2d, out_base_msg.header, target_x, target_y, target_z)
                out_base_msg.detections.append(det3d_base)

            debug_entries.append((det2d, projected, transformed))

        self.objects_3d_camera_pub.publish(out_camera_msg)
        self.objects_3d_base_pub.publish(out_base_msg)

        now = time.time()
        if now - self.last_log_time >= self.log_period:
            self.print_debug_log(
                det2d_msg=msg,
                out_camera_msg=out_camera_msg,
                out_base_msg=out_base_msg,
                debug_entries=debug_entries,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                camera_frame=camera_frame,
                transform_available=(transform is not None),
            )
            self.last_log_time = now

    def get_camera_frame(self, msg: Detection2DArray) -> str:
        # detection header frame이 있으면 그걸 사용함
        # 비어 있으면 camera_frame_fallback을 사용함
        if msg.header.frame_id:
            return msg.header.frame_id
        if self.latest_depth_frame_id:
            return self.latest_depth_frame_id
        return self.camera_frame_fallback

    def lookup_camera_to_target_transform(self, camera_frame: str, stamp):
        if camera_frame == self.target_frame:
            return self.identity_transform()

        try:
            if self.use_latest_tf:
                # latest transform 사용함. static TF 또는 시뮬레이션 초기 단계에서 더 안정적임
                lookup_time = Time()
            else:
                lookup_time = Time.from_msg(stamp)

            # target_frame 기준으로 camera_frame의 점을 변환하기 위한 transform임
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame,
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF lookup failed: {self.target_frame} <- {camera_frame}: {e}"
            )
            return None

    def identity_transform(self):
        # target_frame과 source_frame이 같은 특수 경우용임
        class DummyTranslation:
            x = 0.0
            y = 0.0
            z = 0.0

        class DummyRotation:
            x = 0.0
            y = 0.0
            z = 0.0
            w = 1.0

        class DummyTransform:
            translation = DummyTranslation()
            rotation = DummyRotation()

        class DummyStamped:
            transform = DummyTransform()

        return DummyStamped()

    def transform_point(self, x: float, y: float, z: float, transform_stamped) -> Tuple[float, float, float]:
        # geometry_msgs/TransformStamped를 사용해 3D point를 직접 변환함
        # tf2_geometry_msgs 의존성을 줄이기 위해 quaternion 회전 행렬을 직접 적용함
        t = transform_stamped.transform.translation
        q = transform_stamped.transform.rotation

        r00, r01, r02, r10, r11, r12, r20, r21, r22 = self.quaternion_to_rotation_matrix(
            q.x,
            q.y,
            q.z,
            q.w,
        )

        tx = r00 * x + r01 * y + r02 * z + t.x
        ty = r10 * x + r11 * y + r12 * z + t.y
        tz = r20 * x + r21 * y + r22 * z + t.z

        return float(tx), float(ty), float(tz)

    def quaternion_to_rotation_matrix(self, qx: float, qy: float, qz: float, qw: float):
        # quaternion을 정규화한 뒤 회전 행렬로 바꿈
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-12:
            return 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        r00 = 1.0 - 2.0 * (yy + zz)
        r01 = 2.0 * (xy - wz)
        r02 = 2.0 * (xz + wy)

        r10 = 2.0 * (xy + wz)
        r11 = 1.0 - 2.0 * (xx + zz)
        r12 = 2.0 * (yz - wx)

        r20 = 2.0 * (xz - wy)
        r21 = 2.0 * (yz + wx)
        r22 = 1.0 - 2.0 * (xx + yy)

        return r00, r01, r02, r10, r11, r12, r20, r21, r22

    def get_camera_intrinsics(self) -> Tuple[float, float, float, float]:
        # CameraInfo의 K 행렬에서 pinhole camera intrinsic을 가져옴
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        k = self.latest_camera_info.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])
        return fx, fy, cx, cy

    def project_detection(self, det2d: Detection2D, fx: float, fy: float, cx: float, cy: float) -> Optional[ProjectedObject]:
        # YOLO bbox 중심/크기 읽음
        u_center = float(det2d.bbox.center.position.x)
        v_center = float(det2d.bbox.center.position.y)
        bbox_w = float(det2d.bbox.size_x)
        bbox_h = float(det2d.bbox.size_y)

        depth_image = self.latest_depth
        h, w = depth_image.shape[:2]

        # bbox 좌표 계산 및 이미지 범위로 clipping함
        x1 = u_center - 0.5 * bbox_w
        y1 = v_center - 0.5 * bbox_h
        x2 = u_center + 0.5 * bbox_w
        y2 = v_center + 0.5 * bbox_h

        # bbox 가장자리 배경 영향을 줄이고 싶을 때 shrink_ratio를 사용함
        shrink_x = self.bbox_shrink_ratio * bbox_w
        shrink_y = self.bbox_shrink_ratio * bbox_h
        x1 += shrink_x
        x2 -= shrink_x
        y1 += shrink_y
        y2 -= shrink_y

        x1_i = max(0, int(np.floor(x1)))
        y1_i = max(0, int(np.floor(y1)))
        x2_i = min(w - 1, int(np.ceil(x2)))
        y2_i = min(h - 1, int(np.ceil(y2)))

        if x2_i <= x1_i or y2_i <= y1_i:
            return None

        # bbox 내부 depth crop 샘플링함
        stride = self.bbox_sample_stride
        depth_crop = depth_image[y1_i:y2_i + 1:stride, x1_i:x2_i + 1:stride].astype(np.float32)

        # crop 안의 실제 이미지 좌표 grid를 같이 만들어야 3D point로 역투영할 수 있음
        v_coords, u_coords = np.mgrid[y1_i:y2_i + 1:stride, x1_i:x2_i + 1:stride]
        u_coords = u_coords.astype(np.float32)
        v_coords = v_coords.astype(np.float32)

        if self.latest_depth_encoding == "16UC1":
            # 16UC1은 보통 mm 단위인 경우가 많아서 m 단위로 변환함
            depth_crop = depth_crop / 1000.0

        bbox_total_count = int(depth_crop.size)
        valid_mask = np.isfinite(depth_crop)
        valid_mask &= depth_crop > self.min_depth
        valid_mask &= depth_crop < self.max_depth

        bbox_valid_count = int(np.count_nonzero(valid_mask))
        valid_ratio = bbox_valid_count / float(max(1, bbox_total_count))

        center_depth = self.get_center_depth(u_center, v_center)
        roi_median_depth = self.get_roi_median_depth(u_center, v_center)

        debug_empty = DepthDebugInfo(
            bbox_center_u=u_center,
            bbox_center_v=v_center,
            bbox_width=bbox_w,
            bbox_height=bbox_h,
            center_depth=center_depth,
            roi_median_depth=roi_median_depth,
            foreground_depth=None,
            selected_depth_median=None,
            selected_depth_min=None,
            selected_depth_max=None,
            selected_depth_std=None,
            bbox_valid_count=bbox_valid_count,
            bbox_total_count=bbox_total_count,
            selected_count=0,
            band_min=None,
            band_max=None,
            valid_ratio=valid_ratio,
            selected_ratio=0.0,
        )

        if bbox_valid_count < self.min_valid_points:
            return None

        valid_depths = depth_crop[valid_mask]

        # 핵심 로직:
        # bbox 중심 depth 대신 bbox 내부 valid depth의 foreground percentile을 사용함
        foreground_depth = float(np.percentile(valid_depths, self.foreground_percentile))
        band_min = max(self.min_depth, foreground_depth - self.foreground_band_margin)
        band_max = min(self.max_depth, foreground_depth + self.foreground_band_margin)

        selected_mask = valid_mask & (depth_crop >= band_min) & (depth_crop <= band_max)
        selected_count = int(np.count_nonzero(selected_mask))
        selected_ratio = selected_count / float(max(1, bbox_total_count))

        debug_empty.foreground_depth = foreground_depth
        debug_empty.band_min = band_min
        debug_empty.band_max = band_max
        debug_empty.selected_count = selected_count
        debug_empty.selected_ratio = selected_ratio

        if selected_count < self.min_selected_points:
            return None

        selected_depth = depth_crop[selected_mask]
        selected_u = u_coords[selected_mask]
        selected_v = v_coords[selected_mask]

        # 선택된 foreground band 픽셀들을 camera intrinsic으로 3D point cloud처럼 역투영함
        # optical frame 관례: X 오른쪽, Y 아래쪽, Z 전방 깊이
        x_points = (selected_u - cx) * selected_depth / fx
        y_points = (selected_v - cy) * selected_depth / fy
        z_points = selected_depth

        # mean보다 median이 outlier에 강해서 대표 위치로 median을 사용함
        x = float(np.median(x_points))
        y = float(np.median(y_points))
        z = float(np.median(z_points))

        debug = DepthDebugInfo(
            bbox_center_u=u_center,
            bbox_center_v=v_center,
            bbox_width=bbox_w,
            bbox_height=bbox_h,
            center_depth=center_depth,
            roi_median_depth=roi_median_depth,
            foreground_depth=foreground_depth,
            selected_depth_median=float(np.median(selected_depth)),
            selected_depth_min=float(np.min(selected_depth)),
            selected_depth_max=float(np.max(selected_depth)),
            selected_depth_std=float(np.std(selected_depth)),
            bbox_valid_count=bbox_valid_count,
            bbox_total_count=bbox_total_count,
            selected_count=selected_count,
            band_min=band_min,
            band_max=band_max,
            valid_ratio=valid_ratio,
            selected_ratio=selected_ratio,
        )

        return ProjectedObject(x=x, y=y, z=z, debug=debug)

    def get_center_depth(self, u: float, v: float) -> Optional[float]:
        h, w = self.latest_depth.shape[:2]
        ui = int(round(u))
        vi = int(round(v))

        if ui < 0 or ui >= w or vi < 0 or vi >= h:
            return None

        value = self.latest_depth[vi, ui]
        if self.latest_depth_encoding == "16UC1":
            value = float(value) / 1000.0
        else:
            value = float(value)

        if not np.isfinite(value):
            return None
        if value <= self.min_depth or value >= self.max_depth:
            return None
        return value

    def get_roi_median_depth(self, u: float, v: float) -> Optional[float]:
        h, w = self.latest_depth.shape[:2]
        ui = int(round(u))
        vi = int(round(v))

        if ui < 0 or ui >= w or vi < 0 or vi >= h:
            return None

        r = self.center_roi_radius
        x_min = max(0, ui - r)
        x_max = min(w, ui + r + 1)
        y_min = max(0, vi - r)
        y_max = min(h, vi + r + 1)

        roi = self.latest_depth[y_min:y_max, x_min:x_max].astype(np.float32)
        if self.latest_depth_encoding == "16UC1":
            roi = roi / 1000.0

        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > self.min_depth) & (valid < self.max_depth)]

        if valid.size == 0:
            return None
        return float(np.median(valid))

    def make_detection3d(self, det2d: Detection2D, header: Header, x: float, y: float, z: float) -> Detection3D:
        det3d = Detection3D()
        det3d.header = header

        # class/confidence 그대로 복사함
        for result in det2d.results:
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = result.hypothesis.class_id
            hyp.hypothesis.score = result.hypothesis.score

            # hypothesis pose에도 3D 위치를 넣어둠
            hyp.pose.pose.position.x = x
            hyp.pose.pose.position.y = y
            hyp.pose.pose.position.z = z
            hyp.pose.pose.orientation.w = 1.0

            det3d.results.append(hyp)

        # Detection3D의 bbox 중심에도 3D 위치를 넣음
        # 실제 3D box 크기는 아직 모르므로 size는 0으로 둠
        det3d.bbox.center.position.x = x
        det3d.bbox.center.position.y = y
        det3d.bbox.center.position.z = z
        det3d.bbox.center.orientation.w = 1.0
        det3d.bbox.size.x = 0.0
        det3d.bbox.size.y = 0.0
        det3d.bbox.size.z = 0.0

        return det3d

    def stamp_to_float(self, stamp) -> Optional[float]:
        if stamp is None:
            return None
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def print_debug_log(
        self,
        det2d_msg: Detection2DArray,
        out_camera_msg: Detection3DArray,
        out_base_msg: Detection3DArray,
        debug_entries,
        fx,
        fy,
        cx,
        cy,
        camera_frame: str,
        transform_available: bool,
    ):
        depth_stamp = self.stamp_to_float(self.latest_depth_stamp)
        det_stamp = self.stamp_to_float(det2d_msg.header.stamp)
        info_stamp = self.stamp_to_float(self.latest_camera_info.header.stamp)

        det_depth_dt = None
        det_info_dt = None
        if det_stamp is not None and depth_stamp is not None:
            det_depth_dt = det_stamp - depth_stamp
        if det_stamp is not None and info_stamp is not None:
            det_info_dt = det_stamp - info_stamp

        depth_shape = None if self.latest_depth is None else self.latest_depth.shape

        self.get_logger().info(
            "DEBUG camera/depth/tf | "
            f"depth_encoding={self.latest_depth_encoding}, "
            f"depth_frame={self.latest_depth_frame_id}, "
            f"depth_shape={depth_shape}, "
            f"det_frame={det2d_msg.header.frame_id}, "
            f"camera_frame_used={camera_frame}, target_frame={self.target_frame}, "
            f"tf_available={transform_available}, "
            f"fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}, "
            f"det-depth dt={det_depth_dt}, det-info dt={det_info_dt}"
        )

        if len(out_camera_msg.detections) == 0:
            self.get_logger().info("3D detections: 0")
            return

        for det2d, projected, transformed in debug_entries:
            label = "unknown"
            score = 0.0
            if len(det2d.results) > 0:
                label = det2d.results[0].hypothesis.class_id
                score = det2d.results[0].hypothesis.score

            d = projected.debug

            if transformed.target_x is None:
                target_xyz_str = "None"
            else:
                target_xyz_str = (
                    f"({transformed.target_x:.3f}, "
                    f"{transformed.target_y:.3f}, "
                    f"{transformed.target_z:.3f})"
                )

            self.get_logger().info(
                "3D detection | "
                f"class={label}, score={score:.2f}, "
                f"pixel=({d.bbox_center_u:.1f}, {d.bbox_center_v:.1f}), "
                f"bbox=({d.bbox_width:.1f} x {d.bbox_height:.1f}), "
                f"center_depth={self.format_optional(d.center_depth)}, "
                f"foreground_p{self.foreground_percentile:.0f}={self.format_optional(d.foreground_depth)}, "
                f"band=[{self.format_optional(d.band_min)}, {self.format_optional(d.band_max)}], "
                f"valid={d.bbox_valid_count}/{d.bbox_total_count} ({d.valid_ratio:.2f}), "
                f"selected={d.selected_count}/{d.bbox_total_count} ({d.selected_ratio:.2f}), "
                f"selected_depth_med/min/max/std=({self.format_optional(d.selected_depth_median)}, "
                f"{self.format_optional(d.selected_depth_min)}, {self.format_optional(d.selected_depth_max)}, "
                f"{self.format_optional(d.selected_depth_std)}), "
                f"camera_xyz=({projected.x:.3f}, {projected.y:.3f}, {projected.z:.3f}) m, "
                f"{self.target_frame}_xyz={target_xyz_str} m"
            )

        self.get_logger().info(
            f"Published camera detections={len(out_camera_msg.detections)}, "
            f"{self.target_frame} detections={len(out_base_msg.detections)}"
        )

    def format_optional(self, value: Optional[float]) -> str:
        if value is None:
            return "None"
        return f"{value:.3f}"


def main(args=None):
    rclpy.init(args=args)
    node = Object3DProjectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()