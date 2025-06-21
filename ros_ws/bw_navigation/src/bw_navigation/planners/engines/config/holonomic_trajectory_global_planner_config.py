from dataclasses import dataclass


@dataclass
class HolonomicTrajectoryGlobalPlannerConfig:
    max_velocity: float = 2.5  # m/s
    max_acceleration: float = 3.0  # m/s^2
    wheel_base_radius: float = 0.1  # meters

    replan_interval: float = 0.3  # seconds

    planning_failure_random_noise: float = 0.001
    used_measured_velocity: bool = True
    forward_project_goal_velocity: bool = False
    trajectory_lookahead: float = 0.15  # seconds

    @property
    def max_angular_velocity(self) -> float:
        return self.max_velocity / self.wheel_base_radius

    @property
    def max_acceleration_angular(self) -> float:
        return self.max_acceleration / self.wheel_base_radius
