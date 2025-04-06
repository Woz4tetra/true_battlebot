from enum import Enum, auto


class AppleCollisionEvent(Enum):
    NONE = auto()
    COLLISION = auto()
    NO_MORE_ROOM = auto()
