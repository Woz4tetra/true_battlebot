from __future__ import annotations

from dataclasses import asdict, dataclass

from dacite import from_dict


@dataclass
class EstimationTopicMetadata:
    topic: str
    use_orientation: bool = False
    position_covariance: float = 0.001
    orientation_covariance: float = 0.001

    @classmethod
    def from_dict(cls, data: dict) -> EstimationTopicMetadata:
        return from_dict(data_class=cls, data=data)

    def to_dict(self) -> dict:
        return asdict(self)
