from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from perception_tools.messages.geometry.transform import Pose
from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class PoseWithCovariance:
    pose: Pose
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((6, 6)))

    @classmethod
    def empty(cls) -> PoseWithCovariance:
        return cls(Pose.identity())

    def to_raw(self) -> RawRosMessage:
        return {
            "pose": self.pose.to_raw(),
            "covariance": self.covariance.flatten().tolist(),
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> PoseWithCovariance:
        return cls(
            pose=Pose.from_raw(msg["pose"]),
            covariance=np.array(msg["covariance"]).reshape((6, 6)),
        )
