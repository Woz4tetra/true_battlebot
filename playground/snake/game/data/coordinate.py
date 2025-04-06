from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Coordinate:
    """
    A class representing a coordinate in a 2D grid.

    Attributes:
        x (int): The x-coordinate of the object.
        y (int): The y-coordinate of the object.
    """

    x: int
    y: int

    def __repr__(self) -> str:
        return f"Coordinate(x={self.x}, y={self.y})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Coordinate):
            return NotImplemented
        return self.x == other.x and self.y == other.y

    def __hash__(self) -> int:
        return hash((self.x, self.y))

    def __add__(self, other: Coordinate) -> Coordinate:
        if not isinstance(other, Coordinate):
            return NotImplemented
        return Coordinate(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Coordinate) -> Coordinate:
        if not isinstance(other, Coordinate):
            return NotImplemented
        return Coordinate(self.x - other.x, self.y - other.y)
