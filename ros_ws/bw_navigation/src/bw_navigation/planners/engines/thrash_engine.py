import math

from geometry_msgs.msg import Twist

from bw_navigation.planners.engines.config.thrash_recovery_config import ThrashRecoveryConfig


class ThrashEngine:
    def __init__(self, config: ThrashRecoveryConfig) -> None:
        self.config = config
        self.direction_change_interval = self.config.direction_change_interval
        self.linear_magnitude = -1 * self.config.linear_magnitude
        self.angular_magnitude = self.config.angular_magnitude
        self.prev_direction_change_time = 0.0
        self.now = 0.0

    def reset(self) -> None:
        self.linear_magnitude = -1 * self.config.linear_magnitude
        self.prev_direction_change_time = 0.0
        self.now = 0.0

    def compute(self, dt: float) -> Twist:
        self.now += dt
        if self.now - self.prev_direction_change_time > self.direction_change_interval:
            self.prev_direction_change_time = self.now
            self.linear_magnitude = -1 * self.linear_magnitude
        angular_velocity = self.angular_magnitude * math.sin(self.now * 2 * math.pi / self.direction_change_interval)

        twist = Twist()
        twist.linear.x = self.linear_magnitude
        twist.angular.z = angular_velocity
        return twist
