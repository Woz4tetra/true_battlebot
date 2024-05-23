from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped as RosPoseStamped

from bw_shared.geometry.pose2d import Pose2D
from bw_shared.messages.header import Header


@dataclass(eq=True)
class Pose2DStamped:
    header: Header
    pose: Pose2D

    @classmethod
    def empty(cls) -> Pose2DStamped:
        return cls(
            header=Header.auto(),
            pose=Pose2D.zeros(),
        )

    @classmethod
    def from_msg(cls, msg: RosPoseStamped) -> Pose2DStamped:
        return cls(header=Header.from_msg(msg.header), pose=Pose2D.from_msg(msg.pose))

    def to_msg(self) -> RosPoseStamped:
        return RosPoseStamped(header=self.header.to_msg(), pose=self.pose.to_msg())
