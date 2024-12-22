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

        self.grid_size_px = int(self.config.marker_size * self.config.px_per_meter)

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
        if self.config.flip_grid_horizontal:
            tags_image = self._flip_image_along_horizontal(tags_image)
        image_width_px = math.ceil(self.get_width() * self.config.px_per_meter)
        image_height_px = math.ceil(self.get_height() * self.config.px_per_meter)
        margin_size_px = math.ceil(self.margin_size * self.config.px_per_meter)
        image = np.zeros((image_height_px, image_width_px), dtype=np.uint8)
        image.fill(255)
        image[
            margin_size_px : margin_size_px + tags_image.shape[0],
            margin_size_px : margin_size_px + tags_image.shape[1],
        ] = tags_image
        self._draw_corner_markers(image)

        return image

    def get_ids(self) -> list[int]:
        return self.ids

    def _flip_image_along_vertical(self, image: np.ndarray) -> np.ndarray:
        flipped_image = np.zeros_like(image)
        flipped_image.fill(255)
        for column_num in range(self.config.num_columns):
            flip_column_num = (self.config.num_columns - 1) - column_num
            column_start_orig = int(self.grid_size_px * column_num + self.corner_square_size_px * column_num)
            column_end_orig = column_start_orig + self.grid_size_px
            column_start_flip = int(self.grid_size_px * flip_column_num + self.corner_square_size_px * flip_column_num)
            column_end_flip = column_start_flip + self.grid_size_px
            if column_end_orig > image.shape[0]:
                raise ValueError(
                    f"Computed column size exceeds original image size: {column_end_orig} > {image.shape[0]}"
                )
            flipped_image[:, column_start_flip:column_end_flip] = image[:, column_start_orig:column_end_orig]
        return flipped_image

    def _flip_image_along_horizontal(self, image: np.ndarray) -> np.ndarray:
        flipped_image = np.zeros_like(image)
        flipped_image.fill(255)
        for row_num in range(self.config.num_rows):
            flip_row_num = (self.config.num_rows - 1) - row_num
            row_start_orig = int(self.grid_size_px * row_num + self.corner_square_size_px * row_num)
            row_end_orig = row_start_orig + self.grid_size_px
            row_start_flip = int(self.grid_size_px * flip_row_num + self.corner_square_size_px * flip_row_num)
            row_end_flip = row_start_flip + self.grid_size_px
            if row_end_orig > image.shape[0]:
                raise ValueError(f"Computed row size exceeds original image size: {row_end_orig} > {image.shape[1]}")
            flipped_image[row_start_flip:row_end_flip, :] = image[row_start_orig:row_end_orig, :]
        return flipped_image

    def _draw_corner_markers(self, image: np.ndarray) -> None:
        margin_size_px = math.ceil((self.margin_size - self.corner_square_size) * self.config.px_per_meter)
        for row_num in range(self.config.num_rows + 1):
            for column_num in range(self.config.num_columns + 1):
                square_start_x_px = margin_size_px + (self.grid_size_px + self.corner_square_size_px) * column_num
                square_end_x_px = square_start_x_px + self.corner_square_size_px
                square_start_y_px = margin_size_px + (self.grid_size_px + self.corner_square_size_px) * row_num
                square_end_y_px = square_start_y_px + self.corner_square_size_px
                image[square_start_y_px:square_end_y_px, square_start_x_px:square_end_x_px] = 0

    def get_grid_points(self, anchor: tuple[int, int] = (0, 0)) -> np.ndarray:
        x_size = self.config.num_columns + 1
        y_size = self.config.num_rows + 1
        width = self.all_tag_width
        height = self.all_tag_height
        x_range = ((anchor[0]) * (width / 2), (anchor[0] + 2) * (width / 2))
        y_range = ((anchor[1]) * (height / 2), (anchor[1] + 2) * (height / 2))
        num_90_rotations = (self.config.num_90_rotations + 2) % 4
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
            np.linspace(x_range[0], x_range[1], x_size),
            np.linspace(y_range[0], y_range[1], y_size),
        )
        grid_in_tag = np.vstack([mesh_x.ravel(), mesh_y.ravel()]).reshape(2, -1).T
        zeros = np.zeros((x_size * y_size, 1))
        return np.concatenate((grid_in_tag, zeros), axis=1)
