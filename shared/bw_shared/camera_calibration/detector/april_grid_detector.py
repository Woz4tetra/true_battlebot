import aprilgrid
import cv2
import numpy as np
from cv2 import aruco
from numpy import ndarray

from bw_shared.camera_calibration.board.april_grid_board import AprilGridBoard
from bw_shared.camera_calibration.detector.detection_results import DetectionResults
from bw_shared.camera_calibration.detector.detector import Detector

TAG_FAMILY_MAPPING = {
    "apriltag_36h11": "t36h11",
    "apriltag_25h9": "t25h9",
    "apriltag_16h5": "t16h5",
}


class AprilGridDetector(Detector):
    def __init__(self, board: AprilGridBoard) -> None:
        self.aruco_board = board.aruco_board
        super().__init__(board)
        if self.config.tag_family not in TAG_FAMILY_MAPPING:
            raise ValueError(f"Unsupported tag family for board detection: {self.config.tag_family}")
        tag_family = TAG_FAMILY_MAPPING[self.config.tag_family]
        if self.config.border_bits == 1:
            tag_family += "b1"
        elif self.config.border_bits > 2:
            raise ValueError(f"Unsupported number of border bits: {self.config.border_bits}")
        self.grid_detector = aprilgrid.Detector(tag_family)

    def detect(self, image: ndarray) -> DetectionResults:
        if len(image.shape) > 2:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = self.grid_detector.detect(image)
        corners = []
        tag_ids = []
        for detection in detections:
            corners.append(np.array(detection.corners))
            tag_ids.append(detection.tag_id)
        tag_ids_array = np.array(tag_ids)
        object_points, image_points = self.aruco_board.matchImagePoints(corners, tag_ids_array)
        return DetectionResults(object_points, image_points, corners, tag_ids_array, tag_ids_array)

    def draw_detections(self, debug_image: np.ndarray, results: DetectionResults) -> np.ndarray:
        return aruco.drawDetectedMarkers(debug_image, results.corners, np.array(results.marker_ids), (0, 255, 0))
