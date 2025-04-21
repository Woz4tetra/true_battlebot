from dataclasses import dataclass
from functools import cached_property
from typing import Optional

from bw_interfaces.msg import GoalEngineConfig


@dataclass
class CornerModeConfig:
    max_velocity: float = 0.5
    max_angular_velocity: float = 6.0
    max_acceleration: float = 0.5
    max_centripetal_acceleration: Optional[float] = 0.2
    xy_tolerance: float = 0.05

    @cached_property
    def engine_config(self) -> GoalEngineConfig:
        return GoalEngineConfig(
            max_velocity=self.max_velocity,
            max_angular_velocity=self.max_angular_velocity,
            max_acceleration=self.max_acceleration,
            max_centripetal_acceleration=0.0
            if self.max_centripetal_acceleration is None
            else self.max_centripetal_acceleration,
            is_max_centripetal_acceleration=self.max_centripetal_acceleration is not None,
            rotate_at_end=True,
            end_velocity=0.0,
            is_end_velocity=True,
        )
