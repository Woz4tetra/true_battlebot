from bw_shared.enums.label import OPPONENT_GROUP, ModelLabel

ALL_LABELS = (
    ModelLabel.MINI_BOT,
    ModelLabel.MAIN_BOT,
    ModelLabel.ROBOT,
    ModelLabel.REFEREE,
)

SYNTHETIC_ROBOT_GROUP = OPPONENT_GROUP | {ModelLabel.MRS_BUFF_MK1, ModelLabel.MR_STABS_MK1}
