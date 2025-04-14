from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class ObjectiveType(EnumAutoLowerStr):
    KEYBOARD = auto()
    IDLE = auto()
    AUTO = auto()
    FOLLOW = auto()
    TARGET = auto()
    TELEPORT = auto()
    RELATIVE_TO = auto()
