from typing import Tuple

import cv2
import numpy as np
from tensorflow.lite.python.interpreter import Interpreter


def _softargmax(prob_map: np.ndarray, axis: int = -1) -> float:
    shifted = prob_map - prob_map.max(axis=axis, keepdims=True)
    exp_map = np.exp(shifted)
    weights = exp_map.sum(axis=axis, keepdims=True)
    probs = exp_map / np.clip(weights, 1e-6, None)
    coords = np.arange(prob_map.shape[axis], dtype=np.float32)
    return float(np.tensordot(probs, coords, axes=([axis], [0])))


def _estimate_heading(centerline: np.ndarray) -> float:
    ys = np.linspace(-1.0, 1.0, centerline.shape[0])
    xs = centerline
    x_mean = np.mean(xs)
    y_mean = np.mean(ys)
    num = np.sum((xs - x_mean) * (ys - y_mean))
    den = np.sum((xs - x_mean) ** 2) + 1e-6
    slope = num / den
    return float(np.clip(slope, -0.7, 0.7))


def postprocess_ulfd(
    output_tensor: np.ndarray,
    camera_resolution: Tuple[int, int] = (160, 120),
) -> Tuple[float, float]:
    """
    Convert model output into normalized lane offset and heading.

    Returns:
        lane_offset: normalized lateral offset (-1 = far left, +1 = far right)
        lane_heading: normalized yaw error (-1 = left, +1 = right)
    """
    logits = np.squeeze(output_tensor)
    if logits.ndim == 3 and logits.shape[-1] >= 2:
        center_logits = logits[..., 0]
        orientation_logits = logits[..., 1]
    else:
        center_logits = logits
        orientation_logits = None

    heatmap = cv2.GaussianBlur(center_logits, (5, 5), 0)
    column_profile = heatmap.sum(axis=0)
    if np.all(column_profile == 0):
        return 0.0, 0.0

    col_idx = _softargmax(column_profile[None, :], axis=-1)
    centerline = heatmap.argmax(axis=1)
    if orientation_logits is not None:
        heading = float(np.tanh(np.mean(orientation_logits)))
    else:
        heading = _estimate_heading(centerline / heatmap.shape[1])

    image_width = camera_resolution[0]
    offset_norm = ((col_idx / heatmap.shape[1]) - 0.5) * 2.0
    heading_norm = float(np.clip(heading, -1.0, 1.0))
    return float(np.clip(offset_norm, -1.0, 1.0)), heading_norm


class LaneTFLite:
    """Grayscale lane-segmentation model wrapper."""

    def __init__(self, path: str):
        self.interp = Interpreter(model_path=path, num_threads=2)
        self.interp.allocate_tensors()
        self.i = self.interp.get_input_details()[0]["index"]
        self.o = self.interp.get_output_details()[0]["index"]

    def run(self, bgr: np.ndarray) -> Tuple[float, float]:
        g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        g = cv2.resize(g, (256, 128), interpolation=cv2.INTER_AREA)
        x = g[None, :, :, None].astype(np.float32) / 255.0
        self.interp.set_tensor(self.i, x)
        self.interp.invoke()
        y = self.interp.get_tensor(self.o)
        return postprocess_ulfd(y)