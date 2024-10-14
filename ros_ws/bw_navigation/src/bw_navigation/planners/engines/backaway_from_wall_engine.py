from bw_shared.geometry.input_modulus import normalize_angle
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xy import XY
from geometry_msgs.msg import Twist

from bw_navigation.planners.engines.trajectory_planner_engine_config import BackawayRecoverConfig


class BackawayFromWallEngine:
    def __init__(self, config: BackawayRecoverConfig) -> None:
        self.config = config

    def compute(self, goal_pose: Pose2D, controlled_robot_pose: Pose2D) -> Twist:
        goal_point = XY(goal_pose.x, goal_pose.y)
        controlled_robot_point = XY(controlled_robot_pose.x, controlled_robot_pose.y)
        goal_heading = (goal_point - controlled_robot_point).heading()
        angle_error = normalize_angle(goal_heading - controlled_robot_pose.theta)
        angle_sign = 1 if angle_error > 0 else -1
        linear_sign = 1 if abs(angle_error) < self.config.angle_tolerance else -1
        twist = Twist()
        twist.linear.x = linear_sign * self.config.linear_velocity
        twist.angular.z = angle_sign * self.config.rotate_velocity
        return twist
