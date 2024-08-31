from bw_shared.enums.label import ModelLabel

ALL_LABELS = (
    ModelLabel.MR_STABS_MK1,
    ModelLabel.MR_STABS_MK2,
    ModelLabel.MRS_BUFF_MK1,
    ModelLabel.MRS_BUFF_MK2,
    ModelLabel.ROBOT,
    ModelLabel.REFEREE,
)

MODEL_LABEL_TO_SEGMENTATION_LABEL_MAP = {
    ModelLabel.MR_STABS_MK1: ModelLabel.MINI_BOT,
    ModelLabel.MR_STABS_MK2: ModelLabel.MINI_BOT,
    ModelLabel.MRS_BUFF_MK1: ModelLabel.MAIN_BOT,
    ModelLabel.MRS_BUFF_MK2: ModelLabel.MAIN_BOT,
    ModelLabel.ROBOT: ModelLabel.ROBOT,
    ModelLabel.REFEREE: ModelLabel.REFEREE,
}
