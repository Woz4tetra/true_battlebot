from dataclasses import dataclass

import numpy as np
from cv2 import aruco


@dataclass
class BoardConfig:
    texture_size = 4096
    board_size = 2  # meters
    px_per_meter = texture_size / board_size
    num_rows = 8
    start_id = 1
    marker_size = 0.16
    square_size = 0.24
    border_bits = 1

    def __post_init__(self) -> None:
        self.ids = self.start_id + np.arange(0, self.num_rows * self.num_rows // 2)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.board = aruco.CharucoBoard(
            (self.num_rows, self.num_rows), self.square_size, self.marker_size, self.aruco_dict, ids=self.ids
        )

    def generate_image(self) -> np.ndarray:
        all_marker_size_px = self.num_rows * self.square_size * self.px_per_meter
        margin_size = int(self.texture_size - all_marker_size_px)
        if margin_size < 0:
            raise ValueError("Texture size is too small for the given board size")
        return self.board.generateImage(
            (self.texture_size, self.texture_size), borderBits=self.border_bits, marginSize=margin_size
        )
