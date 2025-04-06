from enum import Enum, auto


class UserInputEvent(Enum):
    """
    Enum representing user input events for the game.
    """

    NONE = auto()
    MOVE_UP = auto()
    MOVE_DOWN = auto()
    MOVE_LEFT = auto()
    MOVE_RIGHT = auto()
    TOGGLE_PAUSE = auto()
    QUIT = auto()
