from enum import Enum


class Label(str, Enum):
    ROBOT = "robot"
    FIELD = "field"
    REFEREE = "referee"
    FRIENDLY_ROBOT = "friendly_robot"
    CONTROLLED_ROBOT = "controlled_robot"
