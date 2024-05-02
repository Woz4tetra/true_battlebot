from __future__ import annotations

from dataclasses import asdict, dataclass

from bw_shared.enums.label import Label
from dacite import from_dict


@dataclass
class LabelColor:
    r: float
    g: float
    b: float
    a: float

    def to_cv_color(self) -> tuple[int, int, int]:
        return (int(self.r * 255), int(self.g * 255), int(self.b * 255))


@dataclass
class ModelMetadata:
    labels: list[Label]
    colors: list[LabelColor]

    @classmethod
    def from_dict(cls, data: dict) -> ModelMetadata:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)
