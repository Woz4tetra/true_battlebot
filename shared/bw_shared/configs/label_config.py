# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from __future__ import annotations

from dataclasses import dataclass, field

from bw_shared.enums.label import Label
from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class LabelConfig:
    name: str
    height: float = 0.0

    def __post_init__(self):
        self.type = Label(self.name)


@dataclass
class LabelsConfig:
    labels: list[LabelConfig] = field(default_factory=lambda: [])

    def __post_init__(self):
        self.labels_map = {label.type: label for label in self.labels}
        if len(self.labels_map) != len(self.labels):
            raise ValueError("Duplicate labels detected")

    @classmethod
    def from_dict(cls, data: dict) -> LabelsConfig:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)

    def get(self, key: Label) -> LabelConfig:
        return self.labels_map[key]
