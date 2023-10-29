from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np
import tf_conversions
from geometry_msgs.msg import Point as RosPoint
from geometry_msgs.msg import Pose as RosPose
from geometry_msgs.msg import Quaternion as RosQuaternion


@dataclass(eq=True)
class Pose2D:
    x: float
    y: float
    theta: float

    @classmethod
    def zeros(cls) -> Pose2D:
        return cls(0.0, 0.0, 0.0)

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    def to_array(self) -> np.ndarray:
        return np.array(self.to_tuple())

    @classmethod
    def from_msg(cls, msg: RosPose) -> Pose2D:
        angles = tf_conversions.transformations.euler_from_quaternion(
            (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        )
        return cls(
            x=msg.position.x,
            y=msg.position.y,
            theta=angles[2],
        )

    def to_msg(self) -> RosPose:
        quat = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        return RosPose(
            position=RosPoint(x=self.x, y=self.y, z=0.0),
            orientation=RosQuaternion(*quat),
        )

    def magnitude(self) -> float:
        return float(np.sqrt(self.x**2 + self.y**2))

    def heading(self) -> float:
        return float(np.arctan2(self.y, self.x))
