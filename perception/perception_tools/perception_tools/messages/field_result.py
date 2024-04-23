from __future__ import annotations

from dataclasses import dataclass

from perception_tools.messages.geometry.transform import Pose
from perception_tools.messages.geometry.xyz import XYZ
from perception_tools.messages.std_msgs.header import Header
from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class FieldResult:
    header: Header
    pose: Pose
    size: XYZ
    type: str = "bw_interfaces/EstimatedObject"

    def to_raw(self) -> RawRosMessage:
        pass

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> FieldResult:
        pass
