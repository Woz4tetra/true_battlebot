from __future__ import annotations

from dataclasses import dataclass, field

from perception_tools.messages.geometry.transform_with_covariance import PoseWithCovariance
from perception_tools.messages.geometry.xyz import XYZ
from perception_tools.messages.std_msgs.header import Header
from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class FieldResult:
    header: Header = field(default_factory=lambda: Header.auto())
    pose: PoseWithCovariance = field(default_factory=lambda: PoseWithCovariance.empty())
    size: XYZ = field(default_factory=lambda: XYZ(0.0, 0.0, 0.0))
    child_frame_id: str = ""
    type: str = "bw_interfaces/EstimatedObject"

    def to_raw(self) -> RawRosMessage:
        return {
            "header": self.header.to_raw(),
            "pose": self.pose.to_raw(),
            "size": self.size.to_raw(),
            "child_frame_id": self.child_frame_id,
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> FieldResult:
        return cls(
            header=Header.from_raw(msg["header"]),
            pose=PoseWithCovariance.from_raw(msg["pose"]),
            size=XYZ.from_raw(msg["size"]),
            child_frame_id=msg["child_frame_id"],
        )
