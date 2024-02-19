from __future__ import annotations

from dataclasses import asdict, dataclass, field

from dacite import from_dict

from bw_tools.enum_auto_lower import EnumAutoLowerStr, auto
from bw_tools.structs.xyz import XYZ


class FieldType(EnumAutoLowerStr):
    NHRL_SMALL = auto()
    NHRL_LARGE = auto()
    MEATBALL_TESTBOX = auto()


@dataclass
class FieldConfig:
    name: str
    size: XYZ

    def __post_init__(self):
        self.type = FieldType(self.name)


@dataclass
class Maps:
    maps: list[FieldConfig] = field(default_factory=lambda: [])

    def __post_init__(self):
        self.maps_map = {field.type: field for field in self.maps}

    @classmethod
    def from_dict(cls, data: dict) -> Maps:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)

    def get(self, key: FieldType) -> FieldConfig:
        return self.maps_map[key]
