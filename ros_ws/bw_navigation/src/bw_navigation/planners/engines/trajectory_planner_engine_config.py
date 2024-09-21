from dataclasses import dataclass
from typing import Optional


@dataclass
class TrajectoryPlannerEngineConfig:
    max_velocity: float = 2.0
    max_acceleration: float = 3.0
    max_centripetal_acceleration: Optional[float] = None
    ramsete_b: float = 2.0
    ramsete_zeta: float = 0.7
    track_width: float = 0.2
    used_measured_velocity: bool = True
    forward_project_goal_velocity: bool = True
    planning_failure_random_noise: float = 0.001

    @property
    def max_angular_velocity(self) -> float:
        return self.max_velocity / self.track_width
