from dataclasses import dataclass, field
from typing import Optional

from bw_shared.pid.config import PidConfig


@dataclass
class PidFollowerEngineConfig:
    linear_pid: PidConfig = PidConfig(kp=3.0, ki=0.0, kd=0.0, kf=1.0)
    angular_pid: PidConfig = PidConfig(kp=6.0, ki=0.0, kd=0.0, kf=1.0)
    always_face_forward: bool = False
    clamp_linear: Optional[tuple[float, float]] = None
    clamp_angular: Optional[tuple[float, float]] = None


@dataclass
class BackawayRecoverConfig:
    linear_velocity: float = 2.0  # m/s
    rotate_velocity: float = 10.0  # rad/s
    angle_tolerance: float = 1.0


@dataclass
class ThrashRecoveryConfig:
    direction_change_interval: float = 0.5
    linear_magnitude: float = 2.0
    angular_magnitude: float = 10.0


@dataclass
class InDangerRecoveryConfig:
    angle_tolerance: float = 0.2
    linear_tolerance: float = 2.5
    size_multiplier: float = 1.5
    velocity_threshold: float = 0.2

    linear_magnitude: float = 2.0
    angular_magnitude: float = 10.0


@dataclass
class TrajectoryPlannerEngineConfig:
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


@dataclass
class RotateToAngleConfig:
    pid: PidConfig = field(default_factory=lambda: PidConfig(kp=3.0, ki=0.0, kd=0.0, kf=1.0))
    timeout: float = 3.0


@dataclass
class RamseteConfig:
    b: float = 4.0
    zeta: float = 0.7


@dataclass
class LocalPlannerEngineConfig:
    obstacle_buffer: float = 0.2  # meters
    obstacle_lookahead: float = 0.5  # meters
    goal_threshold: float = 0.1  # meters


@dataclass
class PlannerConfig:
    move_threshold: float = 0.1  # meters
    move_timeout: float = 1.0  # seconds

    rotate_180_buffer: float = 0.05

    backaway_recover: BackawayRecoverConfig = field(default_factory=BackawayRecoverConfig)
    global_planner: TrajectoryPlannerEngineConfig = field(default_factory=TrajectoryPlannerEngineConfig)
    local_planner: LocalPlannerEngineConfig = field(default_factory=LocalPlannerEngineConfig)
    ramsete: RamseteConfig = field(default_factory=RamseteConfig)
    thrash_recovery: ThrashRecoveryConfig = field(default_factory=ThrashRecoveryConfig)
    in_danger_recovery: InDangerRecoveryConfig = field(default_factory=InDangerRecoveryConfig)
    rotate_to_angle: RotateToAngleConfig = field(default_factory=RotateToAngleConfig)
