import cv2
import numpy as np
from cv2 import aruco

from bw_shared.camera_calibration.board.aruco_board import ArucoBoard
from bw_shared.camera_calibration.detector.detection_results import DetectionResults
from bw_shared.camera_calibration.detector.detector import Detector


class ArucoDetector(Detector):
    def __init__(self, board: ArucoBoard, parameter_path: str) -> None:
        super().__init__(board)
        self.aruco_board = board.aruco_board

        self.detector_params = aruco.DetectorParameters()
        fs = cv2.FileStorage(parameter_path, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_FORMAT_JSON)
        fnode: cv2.FileNode = fs.root()
        self.detector_params.readDetectorParameters(fnode)
        self.detector_params.markerBorderBits = self.config.border_bits


class CharucoDetector(ArucoDetector):
    def __init__(self, board: ArucoBoard, parameter_path: str) -> None:
        super().__init__(board, parameter_path)

        charuco_params = aruco.CharucoParameters()
        refine_params = aruco.RefineParameters()
        if not isinstance(self.aruco_board, aruco.CharucoBoard):
            raise ValueError(f"Supplied board type is not CharucoBoard: {type(self.aruco_board)}")
        self.aruco_detector = aruco.CharucoDetector(
            self.aruco_board, charuco_params, self.detector_params, refine_params
        )

    def detect(self, image: np.ndarray) -> DetectionResults:
        charuco_corners, tag_ids, marker_corners, marker_ids = self.aruco_detector.detectBoard(image)
        corners = [row for row in charuco_corners]
        object_points, image_points = self.aruco_board.matchImagePoints(corners, tag_ids)
        return DetectionResults(object_points, image_points, corners, tag_ids, marker_ids)

    def draw_detections(self, debug_image: np.ndarray, results: DetectionResults) -> np.ndarray:
        return aruco.drawDetectedCornersCharuco(
            debug_image, np.array(results.corners), np.array(results.corner_ids), (0, 255, 0)
        )


class ArucoGridDetector(ArucoDetector):
    def __init__(self, board: ArucoBoard, parameter_path: str) -> None:
        super().__init__(board, parameter_path)

        refine_params = aruco.RefineParameters()
        if not isinstance(self.aruco_board, aruco.GridBoard):
            raise ValueError(f"Supplied board type is not GridBoard: {type(self.aruco_board)}")
        self.aruco_detector = aruco.ArucoDetector(board.aruco_dict, self.detector_params, refine_params)

    def detect(self, image: np.ndarray) -> DetectionResults:
        corners, tag_ids, rejected = self.aruco_detector.detectMarkers(image)
        object_points, image_points = self.aruco_board.matchImagePoints(corners, tag_ids)
        return DetectionResults(object_points, image_points, corners, tag_ids, tag_ids)

    def draw_detections(self, debug_image: np.ndarray, results: DetectionResults) -> np.ndarray:
        return aruco.drawDetectedMarkers(debug_image, results.corners, np.array(results.marker_ids), (0, 255, 0))
