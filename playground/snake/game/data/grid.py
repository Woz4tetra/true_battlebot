from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Grid:
    """
    A class representing a grid for the game.

    Attributes:
        width (int): The width of the grid.
        height (int): The height of the grid.
    """

    width: int
    height: int

    def __repr__(self) -> str:
        return f"Grid(width={self.width}, height={self.height})"

    @property
    def size(self) -> int:
        """
        Get the total size of the grid.

        Returns:
            int: The total size of the grid (width * height).
        """
        return self.width * self.height
