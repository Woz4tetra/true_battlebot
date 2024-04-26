from __future__ import annotations

from dataclasses import dataclass

from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class UVKeypoint:
    x: int
    y: int

    def to_raw(self) -> RawRosMessage:
        return {
            "x": self.x,
            "y": self.y,
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> UVKeypoint:
        return cls(
            x=msg["x"],
            y=msg["y"],
        )
