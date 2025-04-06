from game.data.coordinate import Coordinate
from game.data.direction import Direction


def get_neighbor(coord: Coordinate, direction: Direction) -> Coordinate:
    """
    Get the neighbor coordinate in the specified direction.

    Args:
        coord (Coordinate): The original coordinate.
        direction (Direction): The direction to move.

    Returns:
        Coordinate: The neighbor coordinate in the specified direction.
    """
    return coord + direction.as_coord()
