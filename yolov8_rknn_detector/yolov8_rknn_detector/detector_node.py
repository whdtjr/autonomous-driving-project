from __future__ import annotations

from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .inference_core import Detection, RKNNYoloV8, draw_detections


class YoloV8RKNNDetectorNode(Node):
    """ROS 2 node that wraps RKNN YOLOv8 inference and publishes detections."""

    def __init__(self) -> None:
        super().__init__('yolov8_rknn_detector')

        package_share = Path(get_package_share_directory('yolov8_rknn_detector'))
        default_model_path = package_share / 'models/yolov8n_fp16.rknn'

        self._bridge = CvBridge()
        self._class_names: List[str] = self.declare_parameter(
            'class_names', ['S_Z', 'b_sz', 'schoolzone']
        ).value
        self._publish_debug_image = self.declare_parameter('publish_debug_image', False).value
        self._debug_image_topic = self.declare_parameter('debug_image_topic', 'detections/image').value
        model_path = Path(self.declare_parameter('model_path', str(default_model_path)).value)
        confidence = float(self.declare_parameter('confidence_threshold', 0.6).value)
        nms_threshold = float(self.declare_parameter('nms_threshold', 0.5).value)
        input_width = int(self.declare_parameter('input_width', 640).value)
        input_height = int(self.declare_parameter('input_height', 640).value)
        self._process_every_n = max(1, int(self.declare_parameter('process_every_n', 1).value))
        use_external_norm = bool(self.declare_parameter('use_external_norm', False).value)
        mean = self.declare_parameter('mean', [0.0, 0.0, 0.0]).value
        std = self.declare_parameter('std', [255.0, 255.0, 255.0]).value
        to_rgb = bool(self.declare_parameter('to_rgb', True).value)

        self._detector = RKNNYoloV8(
            model_path=model_path,
            class_names=self._class_names,
            input_size=(input_width, input_height),
            conf_thres=confidence,
            nms_thres=nms_threshold,
            use_external_norm=use_external_norm,
            mean=mean,
            std=std,
            to_rgb=to_rgb,
        )

        image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        detections_topic = self.declare_parameter('detections_topic', 'detections').value

        self._detection_pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self._debug_image_pub = None
        if self._publish_debug_image:
            self._debug_image_pub = self.create_publisher(Image, self._debug_image_topic, 10)

        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data
        )
        self._frame_count = 0
        self.get_logger().info(
            f'Initialized RKNN detector with model: {model_path} | listening to {image_topic}'
        )

    def destroy_node(self) -> bool:
        try:
            self._detector.release()
        except Exception as exc:  # pragma: no cover - release should not fail, but guard anyway
            self.get_logger().warn(f'Failed to release RKNN detector cleanly: {exc}')
        return super().destroy_node()

    def _image_callback(self, msg: Image) -> None:
        self._frame_count += 1
        if (self._frame_count - 1) % self._process_every_n != 0:
            if self._process_every_n > 1:
                self.get_logger().debug(
                    f'Skipping frame {self._frame_count} (process every {self._process_every_n})'
                )
            return

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'CvBridge conversion failed: {exc}')
            return

        try:
            detections, infer_ms = self._detector.infer(cv_image)
        except Exception as exc:
            self.get_logger().error(f'RKNN inference failed: {exc}')
            return

        detection_msg = self._build_detection_array(msg.header, detections)
        detection_msg.header.stamp = msg.header.stamp
        detection_msg.header.frame_id = msg.header.frame_id
        self._detection_pub.publish(detection_msg)

        if detections:
            labels = [
                self._class_names[det.class_id]
                if 0 <= det.class_id < len(self._class_names)
                else str(det.class_id)
                for det in detections
            ]
            self.get_logger().info(
                f'Published {len(detections)} detections in {infer_ms:.2f} ms: {labels}'
            )
        else:
            self.get_logger().debug('No detections in current frame')

        if self._publish_debug_image and self._debug_image_pub is not None:
            debug_img = draw_detections(cv_image, detections, self._class_names)
            try:
                debug_msg = self._bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            except CvBridgeError as exc:
                self.get_logger().warn(f'Failed to convert debug image: {exc}')
            else:
                debug_msg.header = Header()
                debug_msg.header.stamp = msg.header.stamp
                debug_msg.header.frame_id = msg.header.frame_id
                self._debug_image_pub.publish(debug_msg)

    def _build_detection_array(self, header: Header, detections: List[Detection]) -> Detection2DArray:
        array_msg = Detection2DArray()
        array_msg.header = header
        array_msg.detections = []

        for det in detections:
            detection = Detection2D()
            detection.header = header

            bbox = BoundingBox2D()
            bbox.center.position.x = float((det.x1 + det.x2) / 2.0)
            bbox.center.position.y = float((det.y1 + det.y2) / 2.0)
            bbox.center.theta = 0.0
            bbox.size_x = float(max(det.x2 - det.x1, 0.0))
            bbox.size_y = float(max(det.y2 - det.y1, 0.0))
            detection.bbox = bbox

            class_label = (
                self._class_names[det.class_id]
                if 0 <= det.class_id < len(self._class_names)
                else str(det.class_id)
            )
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_label
            hypothesis.hypothesis.score = float(det.score)
            detection.results.append(hypothesis)
            detection.id = class_label
            array_msg.detections.append(detection)

        return array_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloV8RKNNDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down detector node (keyboard interrupt)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
