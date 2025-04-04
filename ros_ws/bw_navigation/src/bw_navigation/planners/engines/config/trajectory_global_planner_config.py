from dataclasses import dataclass
from typing import Optional


@dataclass
class TrajectoryGlobalPlannerConfig:
    max_velocity: float = 6.0  # m/s
    max_acceleration: float = 3.0  # m/s^2
    max_centripetal_acceleration: Optional[float] = 0.75  # m/s^2
    track_width: float = 0.128  # meters

    prediction_magnification: float = 10.0
    replan_interval: float = 0.5

    planning_failure_random_noise: float = 0.001
    used_measured_velocity: bool = True
    forward_project_goal_velocity: bool = True
    trajectory_lookahead: float = 0.1  # seconds

    @property
    def max_angular_velocity(self) -> float:
        return self.max_velocity / self.track_width
