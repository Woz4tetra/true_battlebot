from __future__ import annotations

from dataclasses import dataclass, field

from perception_tools.messages.geometry.transform import Pose
from perception_tools.messages.geometry.xyz import XYZ
from perception_tools.messages.std_msgs.header import Header
from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class FieldResult:
    header: Header = field(default_factory=lambda: Header.auto())
    pose: Pose = field(default_factory=lambda: Pose.identity())
    size: XYZ = field(default_factory=lambda: XYZ(0.0, 0.0, 0.0))
    type: str = "bw_interfaces/EstimatedObject"

    def to_raw(self) -> RawRosMessage:
        pass

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> FieldResult:
        pass
