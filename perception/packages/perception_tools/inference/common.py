import json
from pathlib import Path

import cv2
import numpy as np
import torch
from bw_interfaces.msg import Contour
from bw_shared.enums.label import ModelLabel

from perception_tools.config.model_metadata import ModelMetadata


def load_metadata(metadata_path: str | Path) -> ModelMetadata:
    with open(metadata_path, "r") as file:
        metadata = ModelMetadata.from_dict(json.load(file))
    assert len(metadata.labels) > 0
    return metadata


def mask_to_polygons(mask: np.ndarray, metadata: ModelMetadata) -> dict[ModelLabel, tuple[list[np.ndarray], bool]]:
    # from detectron2 module.
    # cv2.RETR_CCOMP flag retrieves all the contours and arranges them to a 2-level
    # hierarchy. External contours (boundary) of the object are placed in hierarchy-1.
    # Internal contours (holes) are placed in hierarchy-2.
    # cv2.CHAIN_APPROX_NONE flag gets vertices of polygons from contours.
    mask = np.ascontiguousarray(mask).astype(np.uint8)  # some versions of cv2 does not support incontiguous arr
    result = {}
    for layer_index, label in enumerate(metadata.labels):
        if layer_index == 0:  # skip the background
            continue
        layer = (mask == layer_index).astype(np.uint8)
        contour_results = cv2.findContours(layer, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        hierarchy = contour_results[-1]
        if hierarchy is None:  # empty mask
            continue
        has_holes = (hierarchy.reshape(-1, 4)[:, 3] >= 0).sum() > 0
        layer_result = contour_results[-2]
        layer_result = [x.flatten() for x in layer_result]
        # These coordinates from OpenCV are integers in range [0, W-1 or H-1].
        # We add 0.5 to turn them into real-value coordinate space. A better solution
        # would be to first +0.5 and then dilate the returned polygon by 0.5.
        layer_result = [(x + 0.5).reshape(-1, 2) for x in layer_result if len(x) >= 6]

        result[label] = (layer_result, has_holes)
    return result


def get_default_device() -> torch.device:
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def msg_to_mask(contour: Contour, width: int, height: int) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    points = np.array([[point.x, point.y] for point in contour.points], dtype=np.int32)
    mask = cv2.fillPoly(mask, [points], (1,))  # type: ignore
    return np.array(mask != 0)


def multi_msg_to_mask(contours: list[Contour], width: int, height: int) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    for contour in contours:
        sub_mask = msg_to_mask(contour, width, height)
        mask = np.logical_or(mask, sub_mask)
    return mask
