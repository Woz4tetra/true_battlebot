from dataclasses import dataclass

from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.pose2d import Pose2D


def compute_feedback_distance(controlled_robot: EstimatedObject, goal_target: EstimatedObject) -> float:
    robot_pose = Pose2D.from_msg(controlled_robot.pose.pose)
    goal_pose = Pose2D.from_msg(goal_target.pose.pose)
    return goal_pose.relative_to(robot_pose).magnitude()


@dataclass
class GoalProgress:
    is_done: bool
    total_time: float = float("nan")
    time_left: float = float("nan")
    distance_to_goal: float = float("nan")
