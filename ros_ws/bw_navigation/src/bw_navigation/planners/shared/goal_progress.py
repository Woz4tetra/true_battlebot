from dataclasses import dataclass, field

from bw_interfaces.msg import EstimatedObject, GoToGoalFeedback, Trajectory
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
    trajectory: Trajectory = field(default_factory=lambda: Trajectory())

    def fill_feedback(self, feedback: GoToGoalFeedback) -> None:
        feedback.distance_to_goal = self.distance_to_goal
        feedback.total_time = self.total_time
        feedback.time_left = self.time_left
        feedback.trajectory = self.trajectory
