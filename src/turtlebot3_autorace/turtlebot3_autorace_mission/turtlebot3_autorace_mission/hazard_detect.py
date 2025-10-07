#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Codex GPT-5

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rknn.api import RKNN
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


K_INPUT_SIZE = 640
K_NUM_CLASSES = 2
K_LETTERBOX_VALUE = 114


def letterbox(image: np.ndarray) -> np.ndarray:
    """Resize keeping aspect ratio and pad to a square tensor expected by RKNN."""
    height, width = image.shape[:2]
    scale = min(K_INPUT_SIZE / float(width), K_INPUT_SIZE / float(height))

    new_width = int(round(width * scale))
    new_height = int(round(height * scale))
    resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

    output = np.full((K_INPUT_SIZE, K_INPUT_SIZE, 3), K_LETTERBOX_VALUE, dtype=np.uint8)
    x_offset = (K_INPUT_SIZE - new_width) // 2
    y_offset = (K_INPUT_SIZE - new_height) // 2
    output[y_offset:y_offset + new_height, x_offset:x_offset + new_width] = resized
    return output


def decode_output(data: np.ndarray, num_det: int, num_channels: int, conf_thresh: float) -> Tuple[int, float]:
    best_conf = 0.0
    best_class = -1

    if num_channels < 5 + K_NUM_CLASSES:
        return best_class, best_conf

    for index in range(num_det):
        start = index * num_channels
        stop = start + num_channels
        det = data[start:stop]
        obj_conf = det[4]
        if obj_conf < conf_thresh:
            continue

        class_scores = obj_conf * det[5:5 + K_NUM_CLASSES]
        if class_scores.size == 0:
            continue
        cls = int(np.argmax(class_scores))
        score = float(class_scores[cls])
        if score >= conf_thresh and score > best_conf:
            best_conf = score
            best_class = cls

    return best_class, best_conf


class HazardDetect(Node):

    def __init__(self) -> None:
        super().__init__('hazard_detect')

        self.bridge = CvBridge()
        self.rknn: Optional[RKNN] = None
        self.output_details: List[dict] = []

        share_dir = Path(get_package_share_directory('turtlebot3_autorace_mission'))
        default_model = share_dir / 'weights' / 'best_yolov8n_int8.rknn'

        self.model_path = Path(
            self.declare_parameter('model_path', str(default_model)).get_parameter_value().string_value
        )
        self.confidence_threshold = float(
            self.declare_parameter('confidence', 0.25).get_parameter_value().double_value
        )

        self.alert_pub = self.create_publisher(Bool, '/hazard_detector/alert', 10)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_projected',
            self.image_callback,
            10,
        )

        self.initialise_rknn()
        self.get_logger().info(f'hazard_detect node started with model {self.model_path}')

    def initialise_rknn(self) -> None:
        if not self.model_path.is_file():
            raise FileNotFoundError(f'RKNN model not found: {self.model_path}')

        self.rknn = RKNN(verbose=False)
        load_ret = self.rknn.load_rknn(str(self.model_path))
        if load_ret != 0:
            raise RuntimeError(f'Failed to load RKNN model: return code {load_ret}')

        init_ret = self.rknn.init_runtime()
        if init_ret != 0:
            raise RuntimeError(f'Failed to initialise RKNN runtime: return code {init_ret}')

        version_info = self.rknn.get_sdk_version() or {}
        self.get_logger().info(
            'RKNN SDK version: %s driver: %s',
            version_info.get('api_version', 'unknown'),
            version_info.get('drv_version', 'unknown'),
        )

        details = self.rknn.get_output_details()
        if not details:
            raise RuntimeError('RKNN model output information unavailable')

        self.output_details = details
        self.get_logger().info('RKNN context initialised with %d outputs', len(details))

    def image_callback(self, msg: Image) -> None:
        alert = Bool()
        alert.data = False

        if self.rknn is None:
            self.get_logger().error('RKNN context not initialised')
            self.alert_pub.publish(alert)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            input_blob = np.ascontiguousarray(letterbox(rgb_image))
            alert.data = self.run_inference(input_blob)
        except CvBridgeError as exc:
            self.get_logger().error('cv_bridge exception: %s', exc)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('Inference failed: %s', exc)

        self.alert_pub.publish(alert)

    def run_inference(self, input_image: np.ndarray) -> bool:
        if self.rknn is None:
            raise RuntimeError('RKNN context not initialised')

        inputs = [input_image]
        outputs = self.rknn.inference(inputs=inputs)
        if outputs is None:
            raise RuntimeError('RKNN inference returned no outputs')

        hazard_detected = False

        for index, output in enumerate(outputs):
            detail = self.output_details[index] if index < len(self.output_details) else {}
            shape = tuple(output.shape)
            fmt = detail.get('fmt')

            data = np.asarray(output, dtype=np.float32).flatten()

            num_det, num_channels = self._infer_layout(shape, fmt)
            if num_det is None or num_channels is None or num_channels <= 5:
                continue

            cls, confidence = decode_output(data, num_det, num_channels, self.confidence_threshold)
            if cls in (0, 1):
                hazard_detected = True
                self.get_logger().debug(
                    'Detected class %d with confidence %.3f (output %d)',
                    cls,
                    confidence,
                    index,
                )
                break

        return hazard_detected

    def _infer_layout(self, shape: Tuple[int, ...], fmt: Optional[str]) -> Tuple[Optional[int], Optional[int]]:
        if not shape:
            return None, None

        if len(shape) == 4:
            if fmt == 'NCHW' or (fmt is None and shape[1] <= shape[-1]):
                num_channels = shape[1]
                num_det = int(math.prod(shape[2:]))
            else:
                num_channels = shape[-1]
                num_det = int(math.prod(shape[:-1]))
            return num_det, num_channels

        if len(shape) == 3:
            if fmt == 'NCHW' or (fmt is None and shape[0] == 1):
                num_channels = shape[1]
                num_det = int(math.prod(shape[2:]))
                return num_det, num_channels
            if fmt == 'NHWC' or (fmt is None and shape[-1] <= 256):
                num_channels = shape[-1]
                num_det = int(math.prod(shape[:-1]))
                return num_det, num_channels

        if len(shape) == 2:
            first, second = shape
            if second > first:
                return first, second
            return second, first

        if len(shape) == 1:
            return 1, shape[0]

        return None, None

    def destroy_node(self) -> bool:
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = HazardDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
