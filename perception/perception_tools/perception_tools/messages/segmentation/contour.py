from __future__ import annotations

from dataclasses import dataclass

from perception_tools.messages.segmentation.uv_keypoint import UVKeypoint
from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class Contour:
    points: list[UVKeypoint]
    area: float

    def to_raw(self) -> RawRosMessage:
        return {
            "points": [point.to_raw() for point in self.points],
            "area": self.area,
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> Contour:
        return cls(
            points=[UVKeypoint.from_raw(point) for point in msg["points"]],
            area=msg["area"],
        )
