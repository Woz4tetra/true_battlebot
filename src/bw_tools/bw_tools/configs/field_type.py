from dataclasses import dataclass

from bw_tools.enum_auto_lower import EnumAutoLowerStr, auto
from bw_tools.structs.xy import XY


class FieldType(EnumAutoLowerStr):
    NHRL_SMALL = auto()
    NHRL_LARGE = auto()


@dataclass
class FieldConfig:
    size: XY


FIELD_CONFIG = {
    FieldType.NHRL_SMALL: FieldConfig(XY(2.350, 2.350)),
    FieldType.NHRL_LARGE: FieldConfig(XY(4.694, 4.694)),
}
