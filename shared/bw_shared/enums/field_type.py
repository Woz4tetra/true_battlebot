# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto


class FieldType(str, EnumAutoLowerStr):
    NHRL_SMALL = auto()
    NHRL_LARGE = auto()
    MEATBALL_TESTBOX = auto()
    FLOOR = auto()
