from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto
from bw_shared.enums.field_type import FieldType


class CageModel(str, EnumAutoLowerStr):
    DRIVE_TEST_BOX = auto()
    NHRL_3LB_CAGE = auto()
    INVISIBLE_CAGE = auto()


CAGE_MODEL_MAPPING = {
    CageModel.DRIVE_TEST_BOX: FieldType.MEATBALL_TESTBOX,
    CageModel.NHRL_3LB_CAGE: FieldType.NHRL_SMALL,
    CageModel.INVISIBLE_CAGE: FieldType.NHRL_SMALL,
}
