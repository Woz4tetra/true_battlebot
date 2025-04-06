from __future__ import annotations

from dataclasses import dataclass

from game.data.direction import Direction
from game.data.grid_object import GridObject


@dataclass
class SnakePart(GridObject):
    """
    A class representing a part of the snake in the game.

    Inherits from the GridObject class and represents a segment of the snake on the grid.

    Attributes:
        coord (Coordinate): The coordinates of the snake part.
        prev_segment (SnakePart | None): The previous segment of the snake.
    """

    prev_segment: SnakePart | None = None

    def __repr__(self) -> str:
        return f"SnakePart(x={self.x}, y={self.y}, prev_segment={self.prev_segment})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, SnakePart):
            return NotImplemented
        return self.x == other.x and self.y == other.y and self.prev_segment == other.prev_segment

    def __hash__(self) -> int:
        return hash((self.x, self.y, self.prev_segment))


@dataclass
class SnakeHead(SnakePart):
    """
    A class representing the head of the snake in the game.

    Inherits from the SnakePart class and represents the head segment of the snake on the grid.

    Attributes:
        coord (Coordinate): The coordinates of the snake head.
        prev_segment (SnakePart | None): The previous segment of the snake.
        direction (Direction): The current direction of the snake's movement.
    """

    direction: Direction = Direction.NONE

    def __repr__(self) -> str:
        return f"SnakeHead(x={self.x}, y={self.y}, prev_segment={self.prev_segment}, direction={self.direction})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, SnakeHead):
            return NotImplemented
        return (
            self.x == other.x
            and self.y == other.y
            and self.prev_segment == other.prev_segment
            and self.direction == other.direction
        )

    def __hash__(self) -> int:
        return hash((self.x, self.y, self.prev_segment, self.direction))
