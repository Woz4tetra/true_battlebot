# When adding fields to this file, make sure to update the bw_shared_config C++ module as well
from __future__ import annotations

from dataclasses import asdict, dataclass, field

from dacite import from_dict

from bw_shared.enums.label import Label


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
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)

    def get(self, key: Label) -> LabelConfig:
        return self.labels_map[key]
