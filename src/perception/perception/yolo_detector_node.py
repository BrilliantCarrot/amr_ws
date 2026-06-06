#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
)


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")

        # ROS 파라미터 선언함
        # launch/yaml에서 토픽명, 모델명, threshold, FPS 등을 바꿀 수 있게 하기 위함임
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_image_topic", "/detection/image")
        self.declare_parameter("output_objects_topic", "/detection/objects")

        self.declare_parameter("model_path", "yolov8m.pt")
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("device", "0")
        self.declare_parameter("max_fps", 10.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.output_image_topic = self.get_parameter("output_image_topic").value
        self.output_objects_topic = self.get_parameter("output_objects_topic").value

        self.model_path = self.get_parameter("model_path").value
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.image_size = int(self.get_parameter("image_size").value)
        self.device = self.get_parameter("device").value
        self.max_fps = float(self.get_parameter("max_fps").value)

        # ROS Image <-> OpenCV 이미지 변환용 객체임
        self.bridge = CvBridge()

        # YOLO 모델 로드함
        # yolov8m.pt가 없으면 최초 실행 시 ultralytics가 자동 다운로드할 수 있음
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.get_logger().info("YOLO model loaded")

        # Isaac Sim 카메라 토픽은 sensor QoS 성격이 강하므로 BEST_EFFORT로 구독함
        # QoS가 맞지 않으면 토픽은 보이는데 callback이 안 들어오는 문제가 생길 수 있음
        self.sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            self.sensor_qos,
        )

        # 사람이 눈으로 확인할 수 있는 bbox 시각화 이미지 발행함
        self.image_pub = self.create_publisher(
            Image,
            self.output_image_topic,
            10,
        )

        # 다른 ROS2 노드가 사용할 수 있는 구조화된 detection 결과 발행함
        self.objects_pub = self.create_publisher(
            Detection2DArray,
            self.output_objects_topic,
            10,
        )

        self.last_process_time = 0.0
        self.frame_count = 0
        self.last_log_time = time.time()

        self.get_logger().info("YOLO detector node started")
        self.get_logger().info(f"Subscribe: {self.image_topic}")
        self.get_logger().info(f"Publish image  : {self.output_image_topic}")
        self.get_logger().info(f"Publish objects: {self.output_objects_topic}")
        self.get_logger().info(f"Model          : {self.model_path}")
        self.get_logger().info(f"Device         : {self.device}")
        self.get_logger().info(f"Max FPS        : {self.max_fps}")

    def image_callback(self, msg: Image):
        now = time.time()

        # 카메라가 50Hz 정도로 들어오므로 모든 프레임에 YOLO를 돌리면 부담이 큼
        # max_fps로 inference 주기를 제한함
        if self.max_fps > 0.0:
            min_dt = 1.0 / self.max_fps
            if (now - self.last_process_time) < min_dt:
                return

        self.last_process_time = now

        try:
            # ROS Image를 OpenCV BGR 이미지로 변환함
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        try:
            start_time = time.time()

            # YOLO inference 수행함
            results = self.model.predict(
                source=cv_image,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.image_size,
                device=self.device,
                verbose=False,
            )

            inference_time_ms = (time.time() - start_time) * 1000.0
            result = results[0]

            # 1) bbox가 그려진 시각화 이미지 생성 및 발행함
            annotated_image = result.plot()
            out_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            out_img_msg.header = msg.header
            self.image_pub.publish(out_img_msg)

            # 2) ROS2 노드가 사용할 수 있는 Detection2DArray 메시지 생성 및 발행함
            objects_msg = self.build_detection_msg(result, msg.header)
            self.objects_pub.publish(objects_msg)

            self.frame_count += 1

            # 1초마다 처리 FPS, inference time, detection 개수 출력함
            if (now - self.last_log_time) >= 1.0:
                elapsed = now - self.last_log_time
                fps = self.frame_count / elapsed
                num_boxes = len(objects_msg.detections)

                self.get_logger().info(
                    f"YOLO FPS: {fps:.2f}, "
                    f"inference: {inference_time_ms:.1f} ms, "
                    f"detections: {num_boxes}"
                )

                self.frame_count = 0
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

    def build_detection_msg(self, result, header):
        # YOLO 결과를 vision_msgs/Detection2DArray 형식으로 변환함
        # 이 토픽은 W5에서 depth와 결합해 3D 위치 추정에 사용할 예정임
        detections_msg = Detection2DArray()
        detections_msg.header = header

        if result.boxes is None:
            return detections_msg

        names = result.names

        for box in result.boxes:
            # xyxy: 좌상단/우하단 bbox 좌표임
            # x1, y1, x2, y2는 이미지 픽셀 좌표계 기준임
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            class_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())
            class_name = names.get(class_id, str(class_id))

            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            width = x2 - x1
            height = y2 - y1

            detection = Detection2D()
            detection.header = header

            # bbox 중심과 크기를 저장함
            # bbox.center.position.x/y는 이미지 안의 픽셀 좌표임
            detection.bbox.center.position.x = float(cx)
            detection.bbox.center.position.y = float(cy)
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(width)
            detection.bbox.size_y = float(height)

            # class name과 confidence를 저장함
            # class_id에는 "person", "box" 같은 문자열 라벨을 넣음
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_name)
            hypothesis.hypothesis.score = confidence

            detection.results.append(hypothesis)

            detections_msg.detections.append(detection)

        return detections_msg


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()