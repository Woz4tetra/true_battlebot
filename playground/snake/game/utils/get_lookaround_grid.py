from itertools import product

from game.data.coordinate import Coordinate
from game.data.game_container import GameContainer
from game.data.grid_cell_type import GridCellType
from game.utils.get_snake_coords import get_snake_coords
from game.utils.is_in_bounds import is_in_bounds


def get_lookaround_grid(game: GameContainer, look_width: int, look_height: int) -> list[list[GridCellType]]:
    """
    Get a 2D grid representing the look-around area of the snake.
    0 = empty
    1 = snake
    2 = apple
    3 = wall or out of bounds
    """

    if look_width % 2 == 0 or look_height % 2 == 0:
        raise ValueError("look_width and look_height must be odd numbers")
    half_width = look_width // 2
    half_height = look_height // 2
    grid: list[list[GridCellType]] = [[GridCellType.EMPTY for _ in range(look_width)] for _ in range(look_height)]
    snake = game.snake
    apples = game.apples

    snake_coords = set(get_snake_coords(snake))
    apple_coords = {apple.coord for apple in apples}

    for x_offset, y_offset in product(range(look_width), range(look_height)):
        x_absolute = snake.coord.x + x_offset - half_width
        y_absolute = snake.coord.y + y_offset - half_height
        coordinate = Coordinate(x_absolute, y_absolute)

        if not is_in_bounds(coordinate, game.grid):
            grid[y_offset][x_offset] = GridCellType.WALL
        elif coordinate in snake_coords:
            grid[y_offset][x_offset] = GridCellType.SNAKE
        elif coordinate in apple_coords:
            grid[y_offset][x_offset] = GridCellType.APPLE
    return grid
