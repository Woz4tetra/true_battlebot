from dataclasses import dataclass

import numpy as np

from bw_tracking_cam.tag_detection.tag_family import TagFamily


@dataclass
class TagDetection:
    family: TagFamily
    tag_id: int
    hamming: int
    goodness: float
    decision_margin: float
    homography: np.ndarray
    center: np.ndarray
    corners: np.ndarray
