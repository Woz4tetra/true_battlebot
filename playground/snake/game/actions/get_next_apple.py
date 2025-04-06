from game.data.apple import Apple
from game.data.grid import Grid
from game.data.snake import SnakeHead
from game.utils.get_random_coordinate import get_random_coordinate
from game.utils.is_touching_coordinate import is_snake_touching_coordinate, is_touching_coordinate


def get_next_apple(grid: Grid, snake: SnakeHead, apples: list[Apple]) -> Apple | None:
    """
    Get the next apple position that is not occupied by the snake.

    Args:
        grid (Grid): The grid representing the game area.
        snake (SnakeHead): The head of the snake in the game.

    Returns:
        Apple: An apple object with a random coordinate not occupied by the snake.
    """
    attempts = 0
    apple_coords = [apple.coord for apple in apples]
    while attempts < grid.size:
        attempts += 1
        coord = get_random_coordinate(grid)
        if is_snake_touching_coordinate(snake, coord):
            continue
        if is_touching_coordinate(apple_coords, coord):
            continue
        return Apple(coord)
    return None
