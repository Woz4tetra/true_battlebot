from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import List

from bw_shared.enums.label import Label
from dacite import from_dict


@dataclass
class ModelMetadata:
    labels: List[Label]

    @classmethod
    def from_dict(cls, data: dict) -> ModelMetadata:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)
