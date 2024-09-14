from bw_shared.pid.config import PidConfig
from bw_shared.pid.pid import PID
from geometry_msgs.msg import Twist


class RotateToAngleEngine:
    def __init__(self, pid: PidConfig) -> None:
        self.pid = PID(pid)

    def compute(self, dt: float, current_angle: float, goal_angle: float) -> Twist:
        angular_z = self.pid.update(goal_angle, current_angle, dt)
        twist = Twist()
        twist.angular.z = angular_z
        return twist
