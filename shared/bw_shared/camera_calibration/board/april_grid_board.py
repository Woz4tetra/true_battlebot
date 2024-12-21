import math

import numpy as np
from cv2 import aruco

from bw_shared.camera_calibration.board.aruco_board import DICT_MAPPING
from bw_shared.camera_calibration.board.board import Board
from bw_shared.camera_calibration.board_config import BoardConfig


class AprilGridBoard(Board):
    def __init__(self, config: BoardConfig) -> None:
        self.config = config
        aruco_dict = aruco.getPredefinedDictionary(DICT_MAPPING[config.tag_family])
        ids = config.start_id + np.arange(0, config.num_rows * config.num_columns)
        marker_separation = self.config.square_size - self.config.marker_size
        aruco_board = aruco.GridBoard(
            (config.num_columns, config.num_rows),
            config.marker_size,
            marker_separation,
            aruco_dict,
            ids=ids,
        )
        self.ids = ids.tolist()
        self.aruco_board = aruco_board
        self.aruco_dict = aruco_dict

        self.corner_square_size = self.config.square_size - self.config.marker_size
        if self.corner_square_size < 0:
            raise ValueError("Square size is smaller than marker size")
        self.corner_square_size_px = math.ceil(self.corner_square_size * self.config.px_per_meter)

        self.all_tag_width = self._tag_only_dim(self.config.num_columns)
        self.all_tag_height = self._tag_only_dim(self.config.num_rows)

        self.margin_size = (self.config.board_width - self.all_tag_width) / 2
        if self.margin_size < 0:
            raise ValueError("Texture size is too small for the given board size")

        self.image_width = self.all_tag_width + self.margin_size * 2
        self.image_height = self.all_tag_height + self.margin_size * 2

    def get_width(self) -> float:
        return self.image_width

    def get_height(self) -> float:
        return self.image_height

    def _tag_only_dim(self, grid_index: int) -> float:
        corner_sizes = (self.config.square_size - self.config.marker_size) * (grid_index - 1)
        return self.config.marker_size * grid_index + corner_sizes

    def generate_image(self) -> np.ndarray:
        all_tag_width_px = math.ceil(self.all_tag_width * self.config.px_per_meter)
        all_tag_height_px = math.ceil(self.all_tag_height * self.config.px_per_meter)
        tags_image: np.ndarray = self.aruco_board.generateImage(
            (all_tag_width_px, all_tag_height_px),
            borderBits=self.config.border_bits,
        )
        tags_image = np.rot90(tags_image, self.config.num_90_rotations)
        if self.config.flip_grid_vertical:
            tags_image = self._flip_image_along_vertical(tags_image)
        image_width_px = math.ceil(self.get_width() * self.config.px_per_meter)
        image_height_px = math.ceil(self.get_height() * self.config.px_per_meter)
        margin_size_px = math.ceil(self.margin_size * self.config.px_per_meter)
        image = np.zeros((image_height_px, image_width_px), dtype=np.uint8)
        image.fill(255)
        image[
            margin_size_px : margin_size_px + tags_image.shape[0],
            margin_size_px : margin_size_px + tags_image.shape[1],
        ] = tags_image

        return image

    def get_ids(self) -> list[int]:
        return self.ids

    def _flip_image_along_vertical(self, image: np.ndarray) -> np.ndarray:
        column_size_px = int(self.config.marker_size * self.config.px_per_meter)
        flipped_image = np.zeros_like(image)
        flipped_image.fill(255)
        for column_num in range(0, self.config.num_columns):
            flip_column_num = (self.config.num_columns - 1) - column_num
            column_start_orig = int(column_size_px * column_num + self.corner_square_size_px * column_num)
            column_end_orig = column_start_orig + column_size_px
            column_start_flip = int(column_size_px * flip_column_num + self.corner_square_size_px * flip_column_num)
            column_end_flip = column_start_flip + column_size_px
            if column_end_orig > image.shape[0]:
                raise ValueError(
                    f"Computed column size exceeds original image size: {column_end_orig} > {image.shape[0]}"
                )
            flipped_image[:, column_start_flip:column_end_flip] = image[:, column_start_orig:column_end_orig]
        return flipped_image
