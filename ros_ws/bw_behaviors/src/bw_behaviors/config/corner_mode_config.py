from dataclasses import dataclass
from functools import cached_property
from typing import Optional

from bw_interfaces.msg import VelocityProfile


@dataclass
class CornerModeConfig:
    max_velocity: float = 0.5
    max_angular_velocity: float = 6.0
    max_acceleration: float = 0.5
    max_centripetal_acceleration: Optional[float] = None

    @cached_property
    def velocity_profile(self) -> VelocityProfile:
        return VelocityProfile(
            max_velocity=self.max_velocity,
            max_angular_velocity=self.max_angular_velocity,
            max_acceleration=self.max_acceleration,
            max_centripetal_acceleration=float("nan")
            if self.max_centripetal_acceleration is None
            else self.max_centripetal_acceleration,
        )
