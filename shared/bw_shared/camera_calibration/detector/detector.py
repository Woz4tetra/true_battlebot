from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from bw_shared.camera_calibration.board.board import Board
from bw_shared.camera_calibration.detector.detection_results import DetectionResults


class Detector(ABC):
    def __init__(self, board: Board) -> None:
        self.board = board
        self.config = board.config

    @abstractmethod
    def detect(self, image: np.ndarray) -> Optional[DetectionResults]: ...

    @abstractmethod
    def draw_detections(self, debug_image: np.ndarray, results: DetectionResults) -> np.ndarray: ...

    def get_board(self) -> Board:
        return self.board
