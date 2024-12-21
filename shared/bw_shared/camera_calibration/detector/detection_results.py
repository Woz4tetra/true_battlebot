from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np


@dataclass
class DetectionResults:
    object_points: np.ndarray
    image_points: np.ndarray
    corners: Sequence[cv2.typing.MatLike]
    corner_ids: np.ndarray
    marker_ids: np.ndarray

    def __post_init__(self) -> None:
        self.corners_by_id = {
            tag_id: corners for tag_id, corners in zip(self.corner_ids.flatten().tolist(), self.corners)
        }

    def get_corners(self, tag_id: int) -> np.ndarray:
        return self.corners_by_id[tag_id]

    def get_tag_ids(self) -> list[int]:
        return self.marker_ids.flatten().tolist()
