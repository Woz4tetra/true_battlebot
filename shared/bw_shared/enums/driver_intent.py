from enum import IntEnum


class DriverIntent(IntEnum):
    """
    Enum for driver intent.
    """

    NONE = 0
    FOLLOW_ME = 1
    BACK_AWAY = 2
    ATTACK = 3
