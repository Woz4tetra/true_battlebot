from typing import Optional, Tuple

import rospy
from bw_interfaces.msg import EstimatedObject, VelocityProfile
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.pid.config import PidConfig
from geometry_msgs.msg import Twist

from bw_navigation.planners.engines.pid_follower_engine import PidFollowerEngine
from bw_navigation.planners.engines.trajectory_planner_engine_config import PidFollowerEngineConfig
from bw_navigation.planners.goal_progress import GoalProgress, compute_feedback_distance
from bw_navigation.planners.planner_interface import PlannerInterface


class CrashOpponent(PlannerInterface):
    def __init__(self, controlled_robot: str) -> None:
        self.controlled_robot = controlled_robot
        config = PidFollowerEngineConfig(
            linear_pid=PidConfig(kp=3.0, ki=0.0, kd=0.1, kf=1.0),
            angular_pid=PidConfig(kp=5.0, ki=0.01, kd=0.1, kf=0.2),
            always_face_forward=True,
        )
        self.pid_follower = PidFollowerEngine(config)

    def reset(self) -> None:
        pass

    def go_to_goal(
        self,
        dt: float,
        goal_target: EstimatedObject,
        robot_states: dict[str, EstimatedObject],
        field: FieldBounds2D,
        velocity_profile: Optional[VelocityProfile],
    ) -> Tuple[Twist, GoalProgress]:
        if self.controlled_robot not in robot_states:
            rospy.logwarn_throttle(1, f"Robot {self.controlled_robot} not found in robot states")
            return Twist(), GoalProgress(is_done=False)
        controlled_robot = robot_states[self.controlled_robot]
        controlled_robot_pose = Pose2D.from_msg(controlled_robot.pose.pose)
        goal_pose = Pose2D.from_msg(goal_target.pose.pose)
        twist = self.pid_follower.compute(dt, controlled_robot_pose, goal_pose)

        if velocity_profile:
            twist.linear.x = max(
                -1 * velocity_profile.max_velocity,
                min(velocity_profile.max_velocity, twist.linear.x),
            )
            twist.angular.z = max(
                -1 * velocity_profile.max_angular_velocity,
                min(velocity_profile.max_angular_velocity, twist.angular.z),
            )

        return twist, GoalProgress(
            is_done=False, distance_to_goal=compute_feedback_distance(controlled_robot, goal_target)
        )
