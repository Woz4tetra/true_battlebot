from __future__ import annotations

from dataclasses import dataclass

import toml

from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto
from bw_shared.messages.dataclass_utils import from_dict, to_dict


class BoardType(EnumAutoLowerStr):
    CHARUCO = auto()
    ARUCO_GRID = auto()
    APRIL_GRID = auto()


@dataclass
class BoardConfig:
    image_width_px: int = 4096
    board_width: float = 2.0  # meters
    num_rows: int = 8
    num_columns: int = 8
    start_id: int = 1
    marker_size: float = 0.16
    square_size: float = 0.24
    border_bits: int = 1
    board_type: BoardType = BoardType.CHARUCO
    tag_family: str = "6x6_250"
    num_90_rotations: int = 0
    flip_grid_vertical: bool = False
    flip_grid_horizontal: bool = False

    def __post_init__(self) -> None:
        self.px_per_meter = self.image_width_px / self.board_width

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
