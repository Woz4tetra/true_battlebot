from typing import Tuple

import rospy
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.input_modulus import normalize_angle
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.xy import XY
from bw_shared.pid.config import PidConfig
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray

from bw_navigation.planners.engines.rotate_to_angle_engine import RotateToAngleEngine
from bw_navigation.planners.engines.trajectory_planner_engine import TrajectoryPlannerEngine
from bw_navigation.planners.planner_interface import PlannerInterface


class CrashTrajectoryPlanner(PlannerInterface):
    def __init__(
        self,
        controlled_robot: str,
        replan_interval: float = 1.0,
        rotate_180_buffer: float = 0.05,
        angle_tolerance: float = 1.0,
        max_velocity: float = 3.0,
        max_acceleration: float = 1.5,
        ramsete_b: float = 2.0,
        ramsete_zeta: float = 0.7,
    ) -> None:
        self.controlled_robot = controlled_robot
        self.replan_interval = rospy.Duration.from_sec(replan_interval)
        self.rotate_180_buffer = XY(rotate_180_buffer, rotate_180_buffer)
        self.angle_tolerance = angle_tolerance
        self.planner = TrajectoryPlannerEngine(max_velocity, max_acceleration, ramsete_b, ramsete_zeta)
        self.rotate_to_angle = RotateToAngleEngine(PidConfig(kp=6.0, ki=0.01, kd=0.1, kf=0.0))
        self.visualization_publisher = rospy.Publisher(
            "trajectory_visualization", MarkerArray, queue_size=1, latch=True
        )

    def go_to_goal(
        self, dt: float, goal_pose: Pose2D, robot_states: dict[str, EstimatedObject], field: FieldBounds2D
    ) -> Tuple[Twist, bool]:
        if self.controlled_robot not in robot_states:
            rospy.logwarn_throttle(1, f"Robot {self.controlled_robot} not found in robot states")
            return Twist(), False

        controlled_robot_pose = Pose2D.from_msg(robot_states[self.controlled_robot].pose.pose)
        controlled_robot_point = XY(controlled_robot_pose.x, controlled_robot_pose.y)
        goal_point = XY(goal_pose.x, goal_pose.y)

        controlled_robot_size = (
            max(robot_states[self.controlled_robot].size.x, robot_states[self.controlled_robot].size.y) / 2
        )
        buffer = self.rotate_180_buffer + XY(controlled_robot_size, controlled_robot_size)
        inset_field = (field[0] + buffer, field[1] - buffer)
        is_in_bounds = inset_field[0] <= controlled_robot_point <= inset_field[1]
        goal_heading = (goal_point - controlled_robot_point).heading()
        is_in_angle_tolerance = abs(normalize_angle(goal_heading - controlled_robot_pose.theta)) < self.angle_tolerance

        if is_in_bounds or is_in_angle_tolerance:
            if self.planner.should_replan() or rospy.Time.now() - self.planner.start_time > (self.replan_interval):
                rospy.loginfo(f"Replanning trajectory. {controlled_robot_pose} -> {goal_pose}")
                self.planner.generate_trajectory(controlled_robot_pose, goal_pose)
                self.visualization_publisher.publish(self.planner.visualize_trajectory())
            twist = self.planner.compute(controlled_robot_pose)
        else:
            twist = self.rotate_to_angle.compute(dt, controlled_robot_pose.theta, goal_heading)

        return twist, False
