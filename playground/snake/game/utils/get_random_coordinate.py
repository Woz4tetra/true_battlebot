import random

from game.data.coordinate import Coordinate
from game.data.grid import Grid


def get_random_coordinate(grid: Grid) -> Coordinate:
    """
    Get a random coordinate within the bounds of the grid.

    Args:
        grid (Grid): The grid representing the game area.

    Returns:
        Coordinate: A random coordinate within the grid.
    """
    x = random.randint(0, grid.width - 1)
    y = random.randint(0, grid.height - 1)
    return Coordinate(x, y)
