from game.data.coordinate import Coordinate
from game.data.snake import SnakePart


def get_snake_coords(segment: SnakePart) -> list[Coordinate]:
    """
    Iterate through the coordinates of the snake from head to tail.

    Returns:
        list[Coordinate]: A list of coordinates representing the snake's body.
    """
    coords = []
    current_segment: SnakePart | None = segment
    while current_segment is not None:
        coords.append(current_segment.coord)
        current_segment = current_segment.prev_segment
    return coords
