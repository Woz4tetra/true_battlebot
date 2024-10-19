from geometry_msgs.msg import Twist

from bw_navigation.planners.engines.trajectory_planner_engine_config import ThrashRecoveryConfig


class ThrashEngine:
    def __init__(self, config: ThrashRecoveryConfig) -> None:
        self.direction_change_interval = config.direction_change_interval
        self.linear_magnitude = config.linear_magnitude
        self.angular_magnitude = config.angular_magnitude
        self.prev_direction_change_time = 0.0
        self.now = 0.0

    def reset(self) -> None:
        self.prev_direction_change_time = 0.0
        self.now = 0.0

    def compute(self, dt: float) -> Twist:
        self.now += dt
        if self.now - self.prev_direction_change_time > self.direction_change_interval:
            self.prev_direction_change_time = self.now
            self.angular_magnitude = -self.angular_magnitude
            self.linear_magnitude = -self.linear_magnitude

        twist = Twist()
        twist.linear.x = self.linear_magnitude
        twist.angular.z = self.angular_magnitude
        return twist
