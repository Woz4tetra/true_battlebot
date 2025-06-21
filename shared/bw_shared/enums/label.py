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
    OMNI_BOT = auto()
    STAB_RAVE_OMNI = auto()


OUR_ROBOT_GROUP = {
    ModelLabel.MINI_BOT,
    ModelLabel.MR_STABS_MK1,
    ModelLabel.MR_STABS_MK2,
    ModelLabel.MAIN_BOT,
    ModelLabel.MRS_BUFF_MK1,
    ModelLabel.MRS_BUFF_MK2,
    ModelLabel.OMNI_BOT,
    ModelLabel.STAB_RAVE_OMNI,
}
ROBOT_GROUP = OUR_ROBOT_GROUP | {ModelLabel.ROBOT, ModelLabel.OPPONENT_1, ModelLabel.OPPONENT_2}
FRIENDLY_ROBOT_GROUP = OUR_ROBOT_GROUP
OPPONENT_GROUP = {ModelLabel.OPPONENT_1, ModelLabel.OPPONENT_2}
NON_FIELD_GROUP = ROBOT_GROUP | {ModelLabel.REFEREE}
