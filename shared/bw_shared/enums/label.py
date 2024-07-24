# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class Label(str, EnumAutoLowerStr):
    ROBOT = auto()
    FIELD = auto()
    REFEREE = auto()
    FRIENDLY_ROBOT = auto()
    CONTROLLED_ROBOT = auto()
    BACKGROUND = auto()


class ModelLabel(str, EnumAutoLowerStr):
    MR_STABS_MK1 = auto()
    MR_STABS_MK2 = auto()
    MRS_BUFF_MK1 = auto()
    MRS_BUFF_MK2 = auto()
    ROBOT = auto()
    REFEREE = auto()
    FIELD = auto()
    BACKGROUND = auto()
    MAIN_BOT = auto()
    MINI_BOT = auto()
    OPPONENT_1 = auto()
    OPPONENT_2 = auto()
