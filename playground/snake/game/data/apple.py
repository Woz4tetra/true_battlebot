from dataclasses import dataclass

from game.data.grid_object import GridObject


@dataclass
class Apple(GridObject):
    """
    A class representing an apple in the game.

    Inherits from the GridObject class and represents an apple on the grid.

    Attributes:
        x (int): The x-coordinate of the apple.
        y (int): The y-coordinate of the apple.
    """

    def __repr__(self) -> str:
        return f"Apple(x={self.x}, y={self.y})"
