# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from __future__ import annotations

from dataclasses import dataclass, field

from bw_shared.configs.size import Size
from bw_shared.enums.field_type import FieldType
from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class MapConfig:
    name: str
    size: Size

    def __post_init__(self):
        self.type = FieldType(self.name)


@dataclass
class MapsConfig:
    maps: list[MapConfig] = field(default_factory=lambda: [])

    def __post_init__(self):
        self.maps_map = {field.type: field for field in self.maps}
        if len(self.maps_map) != len(self.maps):
            raise ValueError("Duplicate field types in maps")

    @classmethod
    def from_dict(cls, data: dict) -> MapsConfig:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)

    def get(self, key: FieldType) -> MapConfig:
        return self.maps_map[key]
