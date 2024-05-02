from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import List

from dacite import from_dict

from perception_tools.messages.color import Color


@dataclass
class ModelMetadata:
    labels: List[str]
    colors: List[Color]

    @classmethod
    def from_dict(cls, data: dict) -> ModelMetadata:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)
