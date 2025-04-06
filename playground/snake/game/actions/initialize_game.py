from game.actions.get_next_apple import get_next_apple
from game.data.coordinate import Coordinate
from game.data.direction import Direction
from game.data.game_container import GameContainer
from game.data.grid import Grid
from game.data.snake import SnakeHead


def initialize_game(width: int, height: int) -> GameContainer:
    """
    Initialize the game state with a grid, snake, and apples.

    Args:
        width (int): The width of the grid.
        height (int): The height of the grid.

    Returns:
        GameState: The initialized game state.
    """
    grid = Grid(width, height)
    center_x = width // 2
    center_y = height // 2
    snake = SnakeHead(coord=Coordinate(center_x, center_y), direction=Direction.RIGHT)
    first_apple = get_next_apple(grid, snake, [])
    if first_apple is None:
        raise ValueError("No valid apple position found.")
    apples = [first_apple]  # Initialize with one apple at a random position
    start_life = width + height

    return GameContainer(score=0, life=start_life, grid=grid, snake=snake, apples=apples)
