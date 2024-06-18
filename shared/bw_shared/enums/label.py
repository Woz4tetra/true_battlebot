# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class Label(str, EnumAutoLowerStr):
    ROBOT = auto()
    FIELD = auto()
    REFEREE = auto()
    FRIENDLY_ROBOT = auto()
    CONTROLLED_ROBOT = auto()
    BACKGROUND = auto()
