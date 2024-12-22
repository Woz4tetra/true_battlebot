from dataclasses import dataclass

import cv2
import numpy as np
from cv2 import aruco

from bw_tracking_cam.tag_detection.tag_detection import TagDetection
from bw_tracking_cam.tag_detection.tag_family import TagFamily


@dataclass
class ApriltagDetectorConfig:
    tag_family: TagFamily = TagFamily.TAG36H11
    detector_params_path: str = ""
    refine_params_path: str = ""


FAMILY_MAPPING = {
    TagFamily.TAG16H5: aruco.DICT_APRILTAG_16h5,
    TagFamily.TAG25H9: aruco.DICT_APRILTAG_25h9,
    TagFamily.TAG36H10: aruco.DICT_APRILTAG_36h10,
    TagFamily.TAG36H11: aruco.DICT_APRILTAG_36h11,
}


class ApriltagDetector:
    def __init__(self, config: ApriltagDetectorConfig) -> None:
        self.config = config
        detect_params = self.load_detect_params_from_file(config.detector_params_path)
        refine_params = self.load_refine_params_from_file(config.refine_params_path)
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(FAMILY_MAPPING[self.config.tag_family]),
            detect_params,
            refine_params,
        )

    def load_detect_params_from_file(self, parameter_path: str) -> aruco.DetectorParameters:
        if not parameter_path:
            return aruco.DetectorParameters()
        detect_params = aruco.DetectorParameters()
        fs = cv2.FileStorage(parameter_path, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_FORMAT_JSON)
        fnode: cv2.FileNode = fs.root()
        detect_params.readDetectorParameters(fnode)
        return detect_params

    def load_refine_params_from_file(self, parameter_path: str) -> aruco.RefineParameters:
        if not parameter_path:
            return aruco.RefineParameters()
        refine_params = aruco.RefineParameters()
        fs = cv2.FileStorage(parameter_path, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_FORMAT_JSON)
        fnode: cv2.FileNode = fs.root()
        refine_params.readRefineParameters(fnode)
        return refine_params

    def detect(self, image: np.ndarray) -> list[TagDetection]:
        all_corners, ids, rejected = self.detector.detectMarkers(image)
        if ids is None or len(all_corners) == 0:
            return []
        detections = []
        for corners_sample, tag_ids in zip(all_corners, ids):
            corners = corners_sample[0]
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
