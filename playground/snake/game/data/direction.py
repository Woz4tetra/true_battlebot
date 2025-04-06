from enum import Enum, auto

from game.data.coordinate import Coordinate


class Direction(Enum):
    """
    An enumeration representing the possible directions of the snake's movement.

    Attributes:
        UP: Represents upward movement.
        DOWN: Represents downward movement.
        LEFT: Represents leftward movement.
        RIGHT: Represents rightward movement.
    """

    NONE = auto()
    UP = auto()
    DOWN = auto()
    LEFT = auto()
    RIGHT = auto()

    def as_coord(self) -> Coordinate:
        """
        Returns the coordinate change associated with the direction.

        Returns:
            Coordinate: A Coordinate object representing the change in x and y coordinates.
        """
        return {
            Direction.NONE: Coordinate(0, 0),
            Direction.UP: Coordinate(0, -1),
            Direction.DOWN: Coordinate(0, 1),
            Direction.LEFT: Coordinate(-1, 0),
            Direction.RIGHT: Coordinate(1, 0),
        }[self]
