from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import TransformStamped as RosTransformStamped

from bw_shared.geometry.transform3d import Transform3D
from bw_shared.messages.header import Header


@dataclass
class Transform3DStamped:
    header: Header
    child_frame_id: str
    transform: Transform3D

    def to_msg(self) -> RosTransformStamped:
        return RosTransformStamped(
            header=self.header.to_msg(),
            child_frame_id=self.child_frame_id,
            transform=self.transform.to_msg(),
        )

    @classmethod
    def from_msg(cls, msg: RosTransformStamped) -> Transform3DStamped:
        return cls(
            header=Header.from_msg(msg.header),
            child_frame_id=msg.child_frame_id,
            transform=Transform3D.from_msg(msg.transform),
        )
