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

    def get_tag_ids(self) -> list[int]:
        return self.marker_ids.flatten().tolist()
