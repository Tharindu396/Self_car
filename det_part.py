import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import cv2
import numpy as np
from tensorflow.lite.python.interpreter import Interpreter

DetectionBox = Tuple[int, int, int, int]


@dataclass
class Detection:
    """Lightweight container describing a single object detection."""

    class_id: int
    score: float
    box: DetectionBox
    distance_m: float
    lane: str

    def as_dict(self) -> dict:
        return {
            "class_id": int(self.class_id),
            "score": float(self.score),
            "box": tuple(int(v) for v in self.box),
            "distance_m": float(self.distance_m),
            "lane": self.lane,
        }

    @property
    def width(self) -> int:
        return self.box[2] - self.box[0]

    @property
    def height(self) -> int:
        return self.box[3] - self.box[1]

    @property
    def x_center(self) -> float:
        return (self.box[0] + self.box[2]) / 2.0


LANE_BUCKETS = ("left", "center", "right")


def _normalize_boxes(
    boxes: np.ndarray, image_size: Tuple[int, int]
) -> List[DetectionBox]:
    boxes = np.squeeze(boxes)
    if boxes.ndim == 1:
        boxes = boxes[None]
    h, w = image_size[1], image_size[0]
    normalized: List[DetectionBox] = []
    for y_min, x_min, y_max, x_max in boxes:
        x1 = int(np.clip(x_min * w, 0, w - 1))
        y1 = int(np.clip(y_min * h, 0, h - 1))
        x2 = int(np.clip(x_max * w, x1 + 1, w))
        y2 = int(np.clip(y_max * h, y1 + 1, h))
        normalized.append((x1, y1, x2, y2))
    return normalized


def _estimate_distance_m(box: DetectionBox, focal_px: float = 420.0) -> float:
    """Very small pin-hole approximation using box height."""
    pixel_height = max(1, box[3] - box[1])
    real_height_m = 1.6  # assume human sized obstacle
    return float((real_height_m * focal_px) / pixel_height)


def _assign_lane(x_center: float, image_width: int) -> str:
    bucket_width = image_width / len(LANE_BUCKETS)
    idx = int(np.clip(x_center / bucket_width, 0, len(LANE_BUCKETS) - 1))
    return LANE_BUCKETS[idx]


def format_detections(
    boxes: np.ndarray,
    classes: np.ndarray,
    scores: np.ndarray,
    image_size: Tuple[int, int] = (320, 320),
    score_threshold: float = 0.2,
) -> List[Detection]:
    """Convert raw tensors from the TFLite model into structured detections."""
    boxes_px = _normalize_boxes(boxes, image_size)
    classes = np.squeeze(classes).astype(int)
    scores = np.squeeze(scores)

    detections: List[Detection] = []
    image_w = image_size[0]

    for idx, (box, cls_id, score) in enumerate(zip(boxes_px, classes, scores)):
        if score < score_threshold:
            continue
        distance = _estimate_distance_m(box)
        lane = _assign_lane((box[0] + box[2]) / 2.0, image_w)
        detections.append(
            Detection(
                class_id=int(cls_id),
                score=float(score),
                box=box,
                distance_m=distance,
                lane=lane,
            )
        )
    return detections


def filter_by_score(
    detections: Iterable[Detection], min_score: float = 0.35
) -> List[Detection]:
    return [det for det in detections if det.score >= min_score]


def prepare_output(detections: Sequence[Detection]) -> List[dict]:
    """Return serializable objects for downstream DonkeyCar parts."""
    return [det.as_dict() for det in detections]


class DetTFLite:
    """Thin wrapper around a detection TFLite model."""

    def __init__(self, path: str, input_size: Tuple[int, int] = (320, 320)):
        self.nn = Interpreter(model_path=path, num_threads=2)
        self.nn.allocate_tensors()
        self.input_size = input_size
        self.i = self.nn.get_input_details()[0]["index"]
        self.os = [o["index"] for o in self.nn.get_output_details()]

    def run(self, bgr: np.ndarray) -> List[dict]:
        im = cv2.resize(bgr, self.input_size)
        x = im[None].astype(np.uint8)
        self.nn.set_tensor(self.i, x)
        self.nn.invoke()
        tensors = [self.nn.get_tensor(j) for j in self.os]
        if len(tensors) == 4:
            boxes, classes, scores, _ = tensors
        else:
            boxes, classes, scores = tensors[:3]
        detections = filter_by_score(
            format_detections(
                boxes, classes, scores, image_size=self.input_size
            )
        )
        return prepare_output(detections)