from enum import IntEnum


class GridCellType(IntEnum):
    EMPTY = 0
    SNAKE = 1
    APPLE = 2
    WALL = 3
