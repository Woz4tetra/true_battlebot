from dataclasses import dataclass

import rospy
from bw_interfaces.msg import Trajectory as TrajectoryMsg
from bw_shared.geometry.field_bounds import FieldBounds2D
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.twist2d import Twist2D
from geometry_msgs.msg import PoseStamped, TwistStamped
from wpimath import geometry
from wpimath.trajectory import Trajectory, TrajectoryConfig


@dataclass
class PlanningTicket:
    robot_pose: Pose2D
    robot_velocity: Twist2D
    goal_pose: Pose2D
    goal_velocity: Twist2D
    field: FieldBounds2D
    trajectory_config: TrajectoryConfig


@dataclass
class PlanningResult:
    trajectory: Trajectory
    planning_start_time: rospy.Time
    planning_finish_time: rospy.Time


def get_theta(rotation: geometry.Rotation2d) -> float:
    return float(rotation.radians())  # type: ignore


def trajectory_to_msg(start_time: rospy.Time, trajectory: Trajectory) -> TrajectoryMsg:
    msg = TrajectoryMsg()
    msg.header.stamp = start_time
    msg.header.frame_id = "map"
    for state in trajectory.states():
        pose = PoseStamped()
        pose.header.stamp = start_time + rospy.Duration.from_sec(state.t)
        pose.pose = Pose2D(x=state.pose.X(), y=state.pose.Y(), theta=get_theta(state.pose.rotation())).to_msg()
        msg.poses.append(pose)

        twist = TwistStamped()
        twist.header.stamp = start_time + rospy.Duration.from_sec(state.t)
        twist.twist = Twist2D(x=state.velocity, y=0.0, theta=state.velocity * state.curvature).to_msg()
        msg.twists.append(twist)
    return msg
