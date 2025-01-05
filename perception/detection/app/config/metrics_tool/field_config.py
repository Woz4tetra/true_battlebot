from dataclasses import dataclass

from bw_shared.enums.field_type import FieldType


@dataclass
class FieldConfig:
    type: FieldType = FieldType.NHRL_SMALL
    x_buffer: float = 0.1
    y_buffer: float = 0.1
