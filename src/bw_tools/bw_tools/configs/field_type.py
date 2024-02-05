from dataclasses import dataclass

from bw_tools.enum_auto_lower import EnumAutoLowerStr, auto
from bw_tools.structs.xyz import XYZ


class FieldType(EnumAutoLowerStr):
    NHRL_SMALL = auto()
    NHRL_LARGE = auto()


@dataclass
class FieldConfig:
    size: XYZ


FIELD_CONFIG = {
    FieldType.NHRL_SMALL: FieldConfig(XYZ(2.350, 2.350, 1.15)),
    FieldType.NHRL_LARGE: FieldConfig(XYZ(4.694, 4.694, 2.5)),
}
