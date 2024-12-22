import math

import numpy as np
from cv2 import aruco

from bw_shared.camera_calibration.board.board import Board
from bw_shared.camera_calibration.board_config import BoardConfig

DICT_MAPPING = {
    "4x4_50": aruco.DICT_4X4_50,
    "4x4_100": aruco.DICT_4X4_100,
    "4x4_250": aruco.DICT_4X4_250,
    "4x4_1000": aruco.DICT_4X4_1000,
    "5x5_50": aruco.DICT_5X5_50,
    "5x5_100": aruco.DICT_5X5_100,
    "5x5_250": aruco.DICT_5X5_250,
    "5x5_1000": aruco.DICT_5X5_1000,
    "6x6_50": aruco.DICT_6X6_50,
    "6x6_100": aruco.DICT_6X6_100,
    "6x6_250": aruco.DICT_6X6_250,
    "6x6_1000": aruco.DICT_6X6_1000,
    "7x7_50": aruco.DICT_7X7_50,
    "7x7_100": aruco.DICT_7X7_100,
    "7x7_250": aruco.DICT_7X7_250,
    "7x7_1000": aruco.DICT_7X7_1000,
    "aruco_original": aruco.DICT_ARUCO_ORIGINAL,
    "apriltag_16h5": aruco.DICT_APRILTAG_16h5,
    "apriltag_25h9": aruco.DICT_APRILTAG_25h9,
    "apriltag_36h10": aruco.DICT_APRILTAG_36h10,
    "apriltag_36h11": aruco.DICT_APRILTAG_36h11,
    "aruco_mip_36h12": aruco.DICT_ARUCO_MIP_36h12,
}


class ArucoBoard(Board):
    def __init__(self, config: BoardConfig, aruco_board: aruco.Board, aruco_dict: aruco.Dictionary) -> None:
        self.aruco_board = aruco_board
        self.aruco_dict = aruco_dict
        super().__init__(config)

        all_marker_size_px = (
            self.config.num_columns * max(self.config.square_size, self.config.marker_size) * self.config.px_per_meter
        )
        self.margin_size_px = int(self.config.image_width_px - all_marker_size_px) // 2
        if self.margin_size_px < 0:
            raise ValueError("Texture size is too small for the given board size")

        all_tag_width = self.get_width()
        all_tag_height = self.get_height()

        self.image_width_px = math.ceil(all_tag_width * self.config.px_per_meter) + self.margin_size_px * 2
        self.image_height_px = math.ceil(all_tag_height * self.config.px_per_meter) + self.margin_size_px * 2

    def get_width(self) -> float:
        return self.config.num_columns * self.config.square_size

    def get_height(self) -> float:
        return self.config.num_rows * self.config.square_size

    def generate_image(self) -> np.ndarray:
        image: np.ndarray = self.aruco_board.generateImage(
            (self.image_width_px, self.image_height_px),
            borderBits=self.config.border_bits,
            marginSize=self.margin_size_px,
        )
        image = np.rot90(image, self.config.num_90_rotations)
        return image

    def get_grid_points(self, anchor: tuple[int, int] = (0, 0)) -> np.ndarray:
        width = self.get_width()
        height = self.get_height()
        x_range = ((anchor[0]) * (width / 2), (anchor[0] + 2) * (width / 2))
        y_range = ((anchor[1]) * (height / 2), (anchor[1] + 2) * (height / 2))
        num_90_rotations = self.config.num_90_rotations % 4
        if num_90_rotations == 1:
            x_range, y_range = y_range, x_range
        elif num_90_rotations == 2:
            x_range = (-x_range[1], -x_range[0])
            y_range = (-y_range[1], -y_range[0])
        elif num_90_rotations == 3:
            x_range, y_range = y_range, x_range
            x_range = (-x_range[1], -x_range[0])
            y_range = (-y_range[1], -y_range[0])
        mesh_x, mesh_y = np.meshgrid(
            np.linspace(x_range[0], x_range[1], self.config.num_columns + 1),
            np.linspace(y_range[0], y_range[1], self.config.num_rows + 1),
        )
        grid_in_tag = np.vstack([mesh_x.ravel(), mesh_y.ravel()]).reshape(2, -1).T
        zeros = np.zeros((grid_size * grid_size, 1))
        return np.concatenate((grid_in_tag, zeros), axis=1)
