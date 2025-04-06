from dataclasses import dataclass

from game.data.apple import Apple
from game.data.grid import Grid
from game.data.snake import SnakeHead


@dataclass
class GameContainer:
    """
    A class representing the state of the game.

    Attributes:
        score (int): The current score of the game.
        grid (Grid): The grid representing the game area.
        snake (SnakeHead): The head of the snake in the game.
        apples (list[Apple]): A list of apples in the game.
    """

    score: int
    grid: Grid
    snake: SnakeHead
    apples: list[Apple]
