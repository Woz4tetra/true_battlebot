import json

import cv2
import numpy as np
import torch
from bw_interfaces.msg import Contour, UVKeypoint
from bw_shared.enums.label import Label
from perception_tools.config.model_metadata import ModelMetadata


def load_metadata(metadata_path: str) -> ModelMetadata:
    with open(metadata_path, "r") as file:
        metadata = ModelMetadata.from_dict(json.load(file))
    assert len(metadata.labels) > 0
    return metadata


def mask_to_polygons(mask: np.ndarray, metadata: ModelMetadata) -> dict[Label, tuple[np.ndarray, bool]]:
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
        layer = mask == layer_index
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
        layer_result = np.array(
            [x + 0.5 for x in layer_result if len(x) >= 6],
            dtype=np.int32,
        ).reshape(-1, 2)

        result[label] = (layer_result, has_holes)
    return result


def get_default_device() -> torch.device:
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def mask_to_msg(contours: list[np.ndarray]) -> list[Contour]:
    contour_msgs = []
    for contour in contours:
        points = [UVKeypoint(x, y) for x, y in contour]
        area = cv2.contourArea(contour)
        contour_msg = Contour(points=points, area=area)
        contour_msgs.append(contour_msg)
    return contour_msgs
