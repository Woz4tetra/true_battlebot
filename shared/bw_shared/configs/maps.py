from __future__ import annotations

from dataclasses import asdict, dataclass, field

from dacite import from_dict

from bw_shared.configs.size import Size
from bw_shared.enums.field_type import FieldType


@dataclass
class MapConfig:
    name: str
    size: Size

    def __post_init__(self):
        self.type = FieldType(self.name)


@dataclass
class Maps:
    maps: list[MapConfig] = field(default_factory=lambda: [])

    def __post_init__(self):
        self.maps_map = {field.type: field for field in self.maps}

    @classmethod
    def from_dict(cls, data: dict) -> Maps:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)

    def get(self, key: FieldType) -> MapConfig:
        return self.maps_map[key]