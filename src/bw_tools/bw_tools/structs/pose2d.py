from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

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

    def heading(self, other: Optional[Pose2D] = None) -> float:
        if other is None:
            return float(np.arctan2(self.y, self.x))
        else:
            return float(np.arctan2(other.y - self.y, other.x - self.x))

    def to_matrix(self) -> np.ndarray:
        cos_theta = np.cos(self.theta)
        sin_theta = np.sin(self.theta)

        return np.array(
            [
                [cos_theta, -sin_theta, self.x],
                [sin_theta, cos_theta, self.y],
                [0.0, 0.0, 1.0],
            ]
        )

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> Pose2D:
        x = float(matrix[0, 2])
        y = float(matrix[1, 2])
        theta = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
        return cls(x=x, y=y, theta=theta)

    def transform_by(self, other: Pose2D) -> Pose2D:
        """
        Transform this pose by another pose
        """
        tfmat = np.dot(other.to_matrix(), self.to_matrix())
        return Pose2D.from_matrix(tfmat)

    def relative_to(self, other: Pose2D) -> Pose2D:
        """
        Get the pose that transforms from other to self
        """
        tfmat = np.dot(np.linalg.inv(other.to_matrix()), self.to_matrix())
        return Pose2D.from_matrix(tfmat)
