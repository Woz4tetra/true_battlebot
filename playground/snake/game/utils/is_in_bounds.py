from game.data.coordinate import Coordinate
from game.data.grid import Grid


def is_in_bounds(coord: Coordinate, grid: Grid) -> bool:
    """
    Check if the given coordinate is within the bounds of the grid.

    Args:
        coord (Coordinate): The coordinate to check.
        grid (Grid): The grid to check against.

    Returns:
        bool: True if the coordinate is within bounds, False otherwise.
    """
    return 0 <= coord.x < grid.width and 0 <= coord.y < grid.height
