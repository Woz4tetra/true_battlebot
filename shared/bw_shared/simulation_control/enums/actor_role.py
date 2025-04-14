from enum import Enum


class ActorRole(str, Enum):
    EMPTY = ""
    MAIN_BOT = "main_bot"
    MINI_BOT = "mini_bot"
    OPPONENT_1 = "opponent_1"
    REFEREE = "referee"
    CAMERA_1 = "camera_1"
    CAMERA_2 = "camera_2"
