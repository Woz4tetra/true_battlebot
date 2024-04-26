from __future__ import annotations

from dataclasses import dataclass

from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class Empty:
    type: str = "std_msgs/Empty"

    def to_raw(self) -> RawRosMessage:
        return {}

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> Empty:
        return cls()
