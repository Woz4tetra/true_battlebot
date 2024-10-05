import argparse
from typing import Optional, Union

from bw_interfaces.msg import Trajectory
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import PoseStamped, TwistStamped
from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from rosbag.bag import Bag


def plot_x_position(
    axis: Axes,
    poses: Union[list[Optional[PoseStamped]], list[PoseStamped]],
    start_time: float,
    label: str,
    line: str = "-",
    marker="",
) -> None:
    x = [pose.pose.position.x if pose else float("nan") for pose in poses]
    timestamps = [pose.header.stamp.to_sec() - start_time if pose else float("nan") for pose in poses]
    axis.plot(timestamps, x, label=label, linestyle=line, marker=marker)


def plot_y_position(
    axis: Axes,
    poses: Union[list[Optional[PoseStamped]], list[PoseStamped]],
    start_time: float,
    label: str,
    line: str = "-",
    marker="",
) -> None:
    y = [pose.pose.position.y if pose else float("nan") for pose in poses]
    timestamps = [pose.header.stamp.to_sec() - start_time if pose else float("nan") for pose in poses]
    axis.plot(timestamps, y, label=label, linestyle=line, marker=marker)


def plot_yaw_angle(
    axis: Axes,
    poses: Union[list[Optional[PoseStamped]], list[PoseStamped]],
    start_time: float,
    label: str,
    line: str = "-",
    marker="",
) -> None:
    yaw = [Pose2D.from_msg(pose.pose).theta if pose else float("nan") for pose in poses]
    timestamps = [pose.header.stamp.to_sec() - start_time if pose else float("nan") for pose in poses]
    axis.plot(timestamps, yaw, label=label, linestyle=line, marker=marker)


def plot_x_velocity(
    axis: Axes,
    twist: Union[list[Optional[TwistStamped]], list[TwistStamped]],
    start_time: float,
    label: str,
    line: str = "-",
    marker="",
) -> None:
    x = [twist.twist.linear.x if twist else float("nan") for twist in twist]
    timestamps = [pose.header.stamp.to_sec() - start_time if pose else float("nan") for pose in twist]
    axis.plot(timestamps, x, label=label, linestyle=line, marker=marker)


def plot_ang_velocity(
    axis: Axes,
    twists: Union[list[Optional[TwistStamped]], list[TwistStamped]],
    start_time: float,
    label: str,
    line: str = "-",
    marker="",
) -> None:
    ang = [twist.twist.angular.z if twist else float("nan") for twist in twists]
    timestamps = [twist.header.stamp.to_sec() - start_time if twist else float("nan") for twist in twists]
    axis.plot(timestamps, ang, label=label, linestyle=line, marker=marker)


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate filter")
    parser.add_argument("bag", type=str, help="Path to bag")
    args = parser.parse_args()

    recorded_trajectory = Trajectory()
    measured_trajectory = Trajectory()
    commands: list[TwistStamped] = []

    with Bag(args.bag, "r") as bag:
        for topic, msg, timestamp in bag.read_messages():  # type: ignore
            if topic == "recorded_trajectory":
                recorded_trajectory.poses.extend(msg.poses)
                recorded_trajectory.twists.extend(msg.twists)
            elif topic == "measured_trajectory":
                measured_trajectory.poses.extend(msg.poses)
                measured_trajectory.twists.extend(msg.twists)
            elif topic == "commands":
                commands.extend(msg.twists)

    recorded_trajectory.poses.sort(key=lambda pose: pose.header.stamp)
    recorded_trajectory.twists.sort(key=lambda twist: twist.header.stamp)
    measured_trajectory.poses.sort(key=lambda pose: pose.header.stamp)
    measured_trajectory.twists.sort(key=lambda twist: twist.header.stamp)

    fig, axes = plt.subplots(5, 1, sharex=True)
    x_pos_ax, y_pos_ax, yaw_angle_ax, x_vel_ax, ang_vel_ax = axes

    x_pos_ax.set_title("X Position")
    y_pos_ax.set_title("Y Position")
    yaw_angle_ax.set_title("Yaw Angle")
    x_vel_ax.set_title("X Velocity")
    ang_vel_ax.set_title("Angular Velocity")

    ang_vel_ax.set_xlabel("Time (s)")

    start_time = recorded_trajectory.poses[0].header.stamp.to_sec()

    recorded_poses = recorded_trajectory.poses
    measured_poses = measured_trajectory.poses

    plot_x_position(x_pos_ax, recorded_poses, start_time, "Recorded")
    plot_x_position(x_pos_ax, measured_poses, start_time, "Measured")
    plot_y_position(y_pos_ax, recorded_poses, start_time, "Recorded")
    plot_y_position(y_pos_ax, measured_poses, start_time, "Measured")
    plot_yaw_angle(yaw_angle_ax, recorded_poses, start_time, "Recorded")
    plot_yaw_angle(yaw_angle_ax, measured_poses, start_time, "Measured")

    recorded_twists = recorded_trajectory.twists
    measured_twists = measured_trajectory.twists

    plot_x_velocity(x_vel_ax, recorded_twists, start_time, "Recorded")
    plot_x_velocity(x_vel_ax, measured_twists, start_time, "Measured")
    plot_ang_velocity(ang_vel_ax, recorded_twists, start_time, "Recorded")
    plot_ang_velocity(ang_vel_ax, measured_twists, start_time, "Measured")

    plot_x_velocity(x_vel_ax, commands, start_time, "Commanded")
    plot_ang_velocity(ang_vel_ax, commands, start_time, "Commanded")

    x_vel_ax.set_ylim(-4, 4)
    ang_vel_ax.set_ylim(-20, 20)

    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
