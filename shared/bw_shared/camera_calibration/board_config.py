from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import toml

from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto
from bw_shared.messages.dataclass_utils import from_dict, to_dict


class BoardType(EnumAutoLowerStr):
    CHARUCO = auto()
    ARUCO_GRID = auto()
    APRIL_GRID = auto()


@dataclass
class BoardConfig:
    texture_size: int = 4096
    board_size: float = 2.0  # meters
    num_rows: int = 8
    start_id: int = 1
    marker_size: float = 0.16
    square_size: float = 0.24
    border_bits: int = 1
    board_type: BoardType = BoardType.CHARUCO
    tag_family: str = "6x6_250"
    num_90_rotations: int = 0

    def __post_init__(self) -> None:
        self.px_per_meter = self.texture_size / self.board_size
        self.all_tag_width = self.num_rows * self.square_size
        self.grid_points = self.get_grid_points()

    def get_grid_points(self, anchor: tuple[int, int] = (0, 0)) -> np.ndarray:
        grid_size = self.num_rows + 1
        length = self.all_tag_width
        x_range = ((anchor[0]) * (length / 2), (anchor[0] + 2) * (length / 2))
        y_range = ((anchor[1]) * (length / 2), (anchor[1] + 2) * (length / 2))
        num_90_rotations = self.num_90_rotations % 4
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
            np.linspace(x_range[0], x_range[1], grid_size), np.linspace(y_range[0], y_range[1], grid_size)
        )
        grid_in_tag = np.vstack([mesh_x.ravel(), mesh_y.ravel()]).reshape(2, -1).T
        zeros = np.zeros((grid_size * grid_size, 1))
        return np.concatenate((grid_in_tag, zeros), axis=1)

    @classmethod
    def from_dict(cls, data: dict) -> BoardConfig:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)

    @classmethod
    def from_file(cls, path: str) -> BoardConfig:
        with open(path, "r") as file:
            return cls.from_dict(toml.load(file))

    def to_file(self, path: str) -> None:
        with open(path, "w") as file:
            toml.dump(self.to_dict(), file)
