from __future__ import annotations

from dataclasses import dataclass

from game.data.coordinate import Coordinate


@dataclass
class GridObject:
    """
    A class representing a grid object with a position.

    Attributes:
        x (int): The x-coordinate of the object.
        y (int): The y-coordinate of the object.
    """

    coord: Coordinate

    def __repr__(self) -> str:
        return f"GridObject(coord={self.coord})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, GridObject):
            return NotImplemented
        return self.coord == other.coord

    def __hash__(self) -> int:
        return hash(self.coord)

    @property
    def x(self) -> int:
        return self.coord.x

    @property
    def y(self) -> int:
        return self.coord.y
