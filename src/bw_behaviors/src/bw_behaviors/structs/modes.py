from bw_tools.enum_auto_lower import EnumAutoLowerStr, auto


class Mode(EnumAutoLowerStr):
    IDLE = auto()
    CORNER = auto()
    FIGHT = auto()
