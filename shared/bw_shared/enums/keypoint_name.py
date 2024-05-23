from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class KeypointName(str, EnumAutoLowerStr):
    FRONT = auto()
    BACK = auto()


RobotKeypointsNames = [KeypointName.FRONT, KeypointName.BACK]
