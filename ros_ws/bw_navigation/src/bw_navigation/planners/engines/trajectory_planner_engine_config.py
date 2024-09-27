from dataclasses import dataclass, field
from typing import Optional

from bw_shared.pid.config import PidConfig


@dataclass
class PidFollowerEngineConfig:
    linear_pid: PidConfig = PidConfig(kp=3.0, ki=0.0, kd=0.1, kf=0.0)
    angular_pid: PidConfig = PidConfig(kp=6.0, ki=0.01, kd=0.1, kf=0.0)
    always_face_forward: bool = False
    clamp_linear: Optional[tuple[float, float]] = None
    clamp_angular: Optional[tuple[float, float]] = None


@dataclass
class TrajectoryPlannerEngineConfig:
    planning_failure_random_noise: float = 0.001
    used_measured_velocity: bool = True
    forward_project_goal_velocity: bool = True
    trajectory_lookahead: float = 0.05  # seconds


@dataclass
class RamseteConfig:
    b: float = 2.0
    zeta: float = 0.7


@dataclass
class ThrashRecoveryConfig:
    direction_change_interval: float = 0.5
    linear_magnitude: float = 2.0
    angular_magnitude: float = 0.5


@dataclass
class PathPlannerConfig:
    max_velocity: float = 2.0  # m/s
    max_acceleration: float = 1.0  # m/s^2
    max_centripetal_acceleration: Optional[float] = None  # m/s^2
    track_width: float = 0.2  # meters

    move_threshold: float = 0.05  # meters
    move_timeout: float = 1.0  # seconds

    backaway_recover: PidFollowerEngineConfig = field(default_factory=PidFollowerEngineConfig)
    trajectory_planner_engine: TrajectoryPlannerEngineConfig = field(default_factory=TrajectoryPlannerEngineConfig)
    ramsete: RamseteConfig = field(default_factory=RamseteConfig)
    thrash_recovery: ThrashRecoveryConfig = field(default_factory=ThrashRecoveryConfig)

    @property
    def max_angular_velocity(self) -> float:
        return self.max_velocity / self.track_width
