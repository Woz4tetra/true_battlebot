from enum import Enum, auto


class FrontendTickEvent(Enum):
    """
    Enum representing user input events for the game.
    """

    QUIT = auto()
    PAUSED = auto()
    DONE = auto()
    APPLY_INPUT = auto()
    NONE = auto()
