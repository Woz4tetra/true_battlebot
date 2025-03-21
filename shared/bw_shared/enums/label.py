# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class Label(str, EnumAutoLowerStr):
    ROBOT = auto()
    FIELD = auto()
    REFEREE = auto()
    FRIENDLY_ROBOT = auto()
    CONTROLLED_ROBOT = auto()
    BACKGROUND = auto()
    SKIP = ""


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


CONTROLLED_BOT_GROUP = {ModelLabel.MINI_BOT, ModelLabel.MR_STABS_MK1, ModelLabel.MR_STABS_MK2}
MAIN_BOT_GROUP = {ModelLabel.MAIN_BOT, ModelLabel.MRS_BUFF_MK1, ModelLabel.MRS_BUFF_MK2}
ROBOT_GROUP = MAIN_BOT_GROUP | CONTROLLED_BOT_GROUP | {ModelLabel.ROBOT, ModelLabel.OPPONENT_1, ModelLabel.OPPONENT_2}
FRIENDLY_ROBOT_GROUP = MAIN_BOT_GROUP | CONTROLLED_BOT_GROUP
OPPONENT_GROUP = {ModelLabel.OPPONENT_1, ModelLabel.OPPONENT_2}
