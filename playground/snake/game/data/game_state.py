from enum import Enum, auto


class GameState(Enum):
    """
    Enum representing the state of the game.
    """

    RUNNING = auto()
    GAME_OVER = auto()
    GAME_WIN = auto()
