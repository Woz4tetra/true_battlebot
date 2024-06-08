from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum

from bw_shared.enums.label import Label
from dacite import Config, from_dict


@dataclass
class LabelColor:
    r: float
    g: float
    b: float
    a: float = 1.0

    def to_cv_color(self) -> tuple[int, int, int]:
        return (int(self.r * 255), int(self.g * 255), int(self.b * 255))


LABEL_COLORS = {
    Label.CONTROLLED_ROBOT: LabelColor(1.0, 0.0, 0.327),
    Label.FRIENDLY_ROBOT: LabelColor(0.0, 0.115, 1.0),
    Label.REFEREE: LabelColor(0.339, 0.339, 0.339),
    Label.ROBOT: LabelColor(1.0, 0.61, 0.0),
    Label.FIELD: LabelColor(0.0, 1.0, 0.0),
}


@dataclass
class ModelMetadata:
    labels: list[Label]
    colors: list[LabelColor]

    @classmethod
    def from_dict(cls, data: dict) -> ModelMetadata:
        return from_dict(data_class=cls, data=data, config=Config(cast=[Enum]))

    def to_dict(self) -> dict:
        return asdict(self)
