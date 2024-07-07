from dataclasses import dataclass

import numpy as np
from cv2 import aruco

from bw_tools.tag_detection.tag_detection import TagDetection
from bw_tools.tag_detection.tag_family import TagFamily


@dataclass
class ApriltagDetectorConfig:
    tag_family: TagFamily = TagFamily.TAG36H11


FAMILY_MAPPING = {
    TagFamily.TAG16H5: aruco.DICT_APRILTAG_16h5,
    TagFamily.TAG25H9: aruco.DICT_APRILTAG_25h9,
    TagFamily.TAG36H10: aruco.DICT_APRILTAG_36h10,
    TagFamily.TAG36H11: aruco.DICT_APRILTAG_36h11,
}


class ApriltagDetector:
    def __init__(self, config: ApriltagDetectorConfig) -> None:
        self.config = config
        detect_params = aruco.DetectorParameters()
        refine_params = aruco.RefineParameters()
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(FAMILY_MAPPING[self.config.tag_family]),
            detect_params,
            refine_params,
        )

    def detect(self, image: np.ndarray) -> list[TagDetection]:
        all_corners, ids, rejected = self.detector.detectMarkers(image)
        if ids is None or len(all_corners) == 0:
            return []
        detections = []
        for all_corners, tag_ids in zip(all_corners, ids):
            corners = all_corners[0]
            center = corners.mean(axis=0)
            detections.append(
                TagDetection(
                    self.config.tag_family,
                    tag_id=tag_ids[0],
                    hamming=0,
                    goodness=0,
                    decision_margin=0,
                    homography=np.array([]),
                    center=center,
                    corners=corners,
                )
            )
        return detections
