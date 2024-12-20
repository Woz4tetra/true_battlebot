import numpy as np

from bw_shared.camera_calibration.board.board import Board
from bw_shared.camera_calibration.board_config import BoardConfig


class AprilGridBoard(Board):
    def __init__(self, config: BoardConfig) -> None:
        super().__init__(config)

    def generate_image(self) -> np.ndarray:
        raise NotImplementedError
