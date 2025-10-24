import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np
from rknnlite.api import RKNNLite


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    class_id: int


class RKNNYoloV8:
    """Helper that wraps RKNNLite YOLOv8 inference."""

    def __init__(
        self,
        model_path: Path,
        class_names: Sequence[str],
        input_size: Tuple[int, int] = (640, 640),
        conf_thres: float = 0.6,
        nms_thres: float = 0.5,
        use_external_norm: bool = False,
        mean: Sequence[float] = (0.0, 0.0, 0.0),
        std: Sequence[float] = (255.0, 255.0, 255.0),
        to_rgb: bool = True,
    ) -> None:
        self.model_path = Path(model_path)
        self.class_names = list(class_names)
        self.input_size = input_size
        self.conf_thres = conf_thres
        self.nms_thres = nms_thres
        self.use_external_norm = use_external_norm
        self.mean = np.array(mean, dtype=np.float32)
        self.std = np.array(std, dtype=np.float32)
        self.to_rgb = to_rgb
        self.num_classes = len(self.class_names)

        self._rknn = RKNNLite()
        ret = self._rknn.load_rknn(str(self.model_path))
        if ret != 0:
            raise RuntimeError(f'Failed to load RKNN model from {self.model_path}')
        ret = self._rknn.init_runtime()
        if ret != 0:
            raise RuntimeError('Failed to init RKNN runtime')

    def infer(self, image: np.ndarray) -> Tuple[List[Detection], float]:
        """Run inference on an OpenCV image. Returns (detections, inference_ms)."""
        input_data, meta = self._preprocess_image(image)
        start = time.time()
        outputs = self._rknn.inference(inputs=[input_data])
        infer_ms = (time.time() - start) * 1000.0
        boxes, scores, cls_ids = self._decode(outputs)
        detections = self._postprocess_detections(boxes, scores, cls_ids, meta)
        return detections, infer_ms

    def release(self) -> None:
        """Release RKNN runtime resources."""
        self._rknn.release()

    def _preprocess_image(self, img: np.ndarray) -> Tuple[np.ndarray, Dict[str, float]]:
        img_lb, ratio, dw, dh = letterbox(img, self.input_size)
        if self.to_rgb:
            img_lb = cv2.cvtColor(img_lb, cv2.COLOR_BGR2RGB)

        if self.use_external_norm:
            inp = img_lb.astype(np.float32) / 255.0
            inp = (inp - self.mean) / self.std
        else:
            inp = img_lb.astype(np.uint8)

        input_data = np.expand_dims(inp, axis=0)
        meta = {
            'ratio': ratio,
            'dw': dw,
            'dh': dh,
            'orig_shape': img.shape[:2],
        }
        return input_data, meta

    def _decode(self, outputs: Sequence[np.ndarray]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        boxes, scores, cls_ids = decode_yolov8(outputs, self.num_classes, self.conf_thres)
        return boxes, scores, cls_ids

    def _postprocess_detections(
        self,
        boxes: np.ndarray,
        scores: np.ndarray,
        cls_ids: np.ndarray,
        meta: Dict[str, float],
    ) -> List[Detection]:
        if boxes.size == 0:
            return []

        ratio = meta.get('ratio', 1.0)
        dw = meta.get('dw', 0)
        dh = meta.get('dh', 0)
        orig_h, orig_w = meta.get('orig_shape', (0, 0))
        ratio = 1.0 if ratio == 0 else ratio

        boxes = boxes.copy()
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - dw) / ratio
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - dh) / ratio

        if orig_w > 0 and orig_h > 0:
            boxes[:, 0] = np.clip(boxes[:, 0], 0, orig_w - 1)
            boxes[:, 1] = np.clip(boxes[:, 1], 0, orig_h - 1)
            boxes[:, 2] = np.clip(boxes[:, 2], 0, orig_w - 1)
            boxes[:, 3] = np.clip(boxes[:, 3], 0, orig_h - 1)

        keep = nms(boxes, scores, self.nms_thres)
        return [Detection(*boxes[i], float(scores[i]), int(cls_ids[i])) for i in keep]


def letterbox(img: np.ndarray, new_shape: Tuple[int, int], color=(114, 114, 114)):
    h, w = img.shape[:2]
    new_w, new_h = new_shape
    r = min(new_w / w, new_h / h)
    unpad_w, unpad_h = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (unpad_w, unpad_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((new_h, new_w, 3), color, dtype=np.uint8)
    dw, dh = (new_w - unpad_w) // 2, (new_h - unpad_h) // 2
    canvas[dh : dh + unpad_h, dw : dw + unpad_w] = resized
    return canvas, r, dw, dh


def decode_yolov8(
    outputs: Sequence[np.ndarray],
    num_classes: int,
    conf_thres: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    RKNN 출력 형태를 (N, num_classes+4/5) 로 정규화해 후처리합니다.
    형식: [cx, cy, w, h, (obj_conf,) cls_conf_0..]
    """
    out = outputs[0]
    out = np.array(out)

    if out.ndim == 4 and out.shape[0] == 1:
        out = np.squeeze(out, axis=0)
    if out.ndim == 3 and out.shape[0] == 1:
        out = out[0]

    if out.ndim == 3:
        if out.shape[0] in (num_classes + 4, num_classes + 5):
            out = out.transpose(1, 2, 0).reshape(-1, out.shape[0])
        else:
            out = out.reshape(-1, out.shape[-1])
    elif out.ndim == 2 and out.shape[0] in (num_classes + 4, num_classes + 5):
        out = out.T
    elif out.ndim != 2:
        out = out.reshape(out.shape[0], -1).T

    no = out.shape[1]
    if no not in (num_classes + 4, num_classes + 5):
        raise ValueError(f'Unexpected output shape {out.shape}, check model export settings.')

    if no == num_classes + 5:
        obj = out[:, 4:5]
        cls = out[:, 5:]
        cls_id = np.argmax(cls, axis=1)
        cls_score = cls[np.arange(cls.shape[0]), cls_id]
        scores = obj[:, 0] * cls_score
    else:
        cls = out[:, 4:]
        cls_id = np.argmax(cls, axis=1)
        scores = cls[np.arange(cls.shape[0]), cls_id]

    mask = scores > conf_thres
    out = out[mask]
    scores = scores[mask]
    cls_id = cls_id[mask]

    boxes = out[:, :4].copy()
    boxes[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    boxes[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    boxes[:, 2] = boxes[:, 0] + boxes[:, 2]
    boxes[:, 3] = boxes[:, 1] + boxes[:, 3]
    return boxes, scores, cls_id


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thres: float = 0.45):
    if len(boxes) == 0:
        return []
    xywh = boxes.copy()
    xywh[:, 2] = xywh[:, 2] - xywh[:, 0]
    xywh[:, 3] = xywh[:, 3] - xywh[:, 1]
    indices = cv2.dnn.NMSBoxes(
        bboxes=xywh.tolist(),
        scores=scores.tolist(),
        score_threshold=0.0,
        nms_threshold=iou_thres,
    )
    if len(indices) == 0:
        return []
    return indices.flatten().tolist()


def draw_detections(image: np.ndarray, detections: Sequence[Detection], class_names: Sequence[str]):
    vis = image.copy()
    for det in detections:
        x1, y1, x2, y2 = map(int, [det.x1, det.y1, det.x2, det.y2])
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = class_names[det.class_id] if 0 <= det.class_id < len(class_names) else str(det.class_id)
        cv2.putText(
            vis,
            f'{label}:{det.score:.2f}',
            (x1, max(0, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
    return vis
