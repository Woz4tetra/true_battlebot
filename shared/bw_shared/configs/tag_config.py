from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from bw_interfaces.msg import TagConfigMsg
from geometry_msgs.msg import Vector3

from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D


@dataclass
class TagConfig:
    tag_id: int
    tag_size: float
    x: float = 0.0  # meters
    y: float = 0.0  # meters
    z: float = 0.0  # meters
    roll: float = 0.0  # degrees
    pitch: float = 0.0  # degrees
    yaw: float = 0.0  # degrees

    @property
    def transform(self) -> Transform3D:
        return Transform3D.from_position_and_rpy(
            Vector3(self.x, self.y, self.z),
            RPY(
                (
                    np.deg2rad(self.roll),
                    np.deg2rad(self.pitch),
                    np.deg2rad(self.yaw),
                )
            ),
        )

    @property
    def tag_corners(self) -> np.ndarray:
        s = self.tag_size / 2
        return np.array(
            [
                [-s, -s, 0.0, 1.0],
                [s, -s, 0.0, 1.0],
                [s, s, 0.0, 1.0],
                [-s, s, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

    @property
    def bundle_corners(self) -> np.ndarray:
        corners_bundle_space = np.dot(self.transform.tfmat, self.tag_corners.T).T
        return corners_bundle_space[:, :3]

    @classmethod
    def from_msg(cls, msg: TagConfigMsg) -> TagConfig:
        return cls(
            tag_id=msg.tag_id,
            tag_size=msg.tag_size,
            x=msg.x,
            y=msg.y,
            z=msg.z,
            roll=msg.roll,
            pitch=msg.pitch,
            yaw=msg.yaw,
        )

    def to_msg(self) -> TagConfigMsg:
        return TagConfigMsg(
            tag_id=self.tag_id,
            tag_size=self.tag_size,
            x=self.x,
            y=self.y,
            z=self.z,
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw,
        )


BundleConfig = list[TagConfig]
