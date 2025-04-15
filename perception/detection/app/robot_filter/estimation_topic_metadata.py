from __future__ import annotations

from dataclasses import dataclass

from bw_shared.messages.dataclass_utils import from_dict, to_dict


@dataclass
class EstimationTopicMetadata:
    topic: str
    use_orientation: bool = False
    position_covariance: float = 0.001
    orientation_covariance: float = 0.001

    @classmethod
    def from_dict(cls, data: dict) -> EstimationTopicMetadata:
        return from_dict(cls, data)

    def to_dict(self) -> dict:
        return to_dict(self)
