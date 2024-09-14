from typing import Tuple

import rospy
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray

from bw_navigation.planners.engines.trajectory_planner_engine import TrajectoryPlannerEngine
from bw_navigation.planners.planner_interface import PlannerInterface


class CrashTrajectoryPlanner(PlannerInterface):
    def __init__(self, controlled_robot: str, replan_interval: float = 1.0) -> None:
        self.controlled_robot = controlled_robot
        self.replan_interval = rospy.Duration.from_sec(replan_interval)
        self.planner = TrajectoryPlannerEngine()
        self.visualization_publisher = rospy.Publisher(
            "trajectory_visualization", MarkerArray, queue_size=1, latch=True
        )

    def go_to_goal(
        self, dt: float, goal_pose: Pose2D, robot_states: dict[str, EstimatedObject], field: EstimatedObject
    ) -> Tuple[Twist, bool]:
        if self.controlled_robot not in robot_states:
            rospy.logwarn_throttle(1, f"Robot {self.controlled_robot} not found in robot states")
            return Twist(), False
        controlled_robot_pose = Pose2D.from_msg(robot_states[self.controlled_robot].pose.pose)
        if self.planner.should_replan() or rospy.Time.now() - self.planner.start_time > (self.replan_interval):
            rospy.loginfo(f"Replanning trajectory. {controlled_robot_pose} -> {goal_pose}")
            self.planner.generate_trajectory(controlled_robot_pose, goal_pose)
            self.visualization_publisher.publish(self.planner.visualize_trajectory())
        twist = self.planner.compute(controlled_robot_pose)
        return twist, False
