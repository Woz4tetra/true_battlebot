from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from functools import cached_property

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
    Label.BACKGROUND: LabelColor(0.0, 0.0, 0.0),
    Label.CONTROLLED_ROBOT: LabelColor(1.0, 0.0, 0.327),
    Label.FRIENDLY_ROBOT: LabelColor(0.0, 0.115, 1.0),
    Label.REFEREE: LabelColor(0.339, 0.339, 0.339),
    Label.ROBOT: LabelColor(1.0, 0.61, 0.0),
    Label.FIELD: LabelColor(0.0, 1.0, 0.0),
}


@dataclass
class ModelMetadata:
    labels: list[Label]
    colors: list[LabelColor] = field(default_factory=list)

    def __post_init__(self):
        if not self.colors:
            self.colors = [LABEL_COLORS[label] for label in self.labels]

    @classmethod
    def from_dict(cls, data: dict) -> ModelMetadata:
        return from_dict(data_class=cls, data=data, config=Config(cast=[Enum]))

    def to_dict(self) -> dict:
        return asdict(self)

    @cached_property
    def color_map(self) -> dict[Label, LabelColor]:
        return dict(zip(self.labels, self.colors))


FIELD_SEMANTIC_MODEL_METADATA = ModelMetadata(
    labels=[Label.BACKGROUND, Label.FIELD],
)
