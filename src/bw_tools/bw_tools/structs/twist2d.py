from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np
from geometry_msgs.msg import Twist as RosTwist
from geometry_msgs.msg import Vector3 as RosVector3


@dataclass(eq=True)
class Twist2D:
    x: float
    y: float
    theta: float

    @classmethod
    def zeros(cls) -> Twist2D:
        return cls(0.0, 0.0, 0.0)

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    def to_array(self) -> np.ndarray:
        return np.array(self.to_tuple())

    @classmethod
    def from_msg(cls, msg: RosTwist) -> Twist2D:
        return cls(x=msg.linear.x, y=msg.linear.y, theta=msg.angular.z)

    def to_msg(self) -> RosTwist:
        return RosTwist(
            linear=RosVector3(x=self.x, y=self.y, z=0.0),
            angular=RosVector3(x=0.0, y=0.0, z=self.theta),
        )

    def speed(self) -> float:
        return float(np.sqrt(self.x**2 + self.y**2))

    def magnitude(self) -> float:
        return self.speed()

    def heading(self) -> float:
        return float(np.arctan2(self.y, self.x))

    def almost_equal(self, other: Twist2D, rtol: float = 1.0e-5, atol: float = 1.0e-8) -> bool:
        position_close = np.allclose(self.to_array()[0:2], other.to_array()[0:2], rtol, atol)
        this_theta = self.theta % (2 * np.pi)
        other_theta = other.theta % (2 * np.pi)
        theta_close = np.allclose(this_theta, other_theta, rtol, atol)
        return position_close and theta_close
