import argparse
from typing import Any, Optional, Union

from bw_interfaces.msg import Trajectory
from bw_shared.geometry.pose2d import Pose2D
from geometry_msgs.msg import PoseStamped, TwistStamped
from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from rosbag.bag import Bag


def expand_list(lst: list, size: int, fill: Any = None) -> list:
    return lst + [fill] * (size - len(lst))


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
    parser = argparse.ArgumentParser(description="Evaluate trajectories")
    parser.add_argument("bag", type=str, help="Path to bag")
    args = parser.parse_args()

    expected_trajectories: list[list[Trajectory]] = []
    recorded_trajectories: list[Trajectory] = []
    measured_trajectories: list[Trajectory] = []
    commands: list[TwistStamped] = []

    with Bag(args.bag, "r") as bag:
        for topic, msg, timestamp in bag.read_messages():  # type: ignore
            if topic == "expected_trajectory":
                expected_trajectories = expand_list(expected_trajectories, msg.header.seq + 1, [])
                expected_trajectories[msg.header.seq].append(msg)
            elif topic == "recorded_trajectory":
                recorded_trajectories = expand_list(recorded_trajectories, msg.header.seq + 1)
                recorded_trajectories[msg.header.seq] = msg
            elif topic == "measured_trajectory":
                measured_trajectories = expand_list(measured_trajectories, msg.header.seq + 1)
                measured_trajectories[msg.header.seq] = msg
            elif topic == "commands":
                commands.extend(msg.twists)

    fig, axes = plt.subplots(5, 1, sharex=True)
    x_pos_ax, y_pos_ax, yaw_angle_ax, x_vel_ax, ang_vel_ax = axes

    x_pos_ax.set_title("X Position")
    y_pos_ax.set_title("Y Position")
    yaw_angle_ax.set_title("Yaw Angle")
    x_vel_ax.set_title("X Velocity")
    ang_vel_ax.set_title("Angular Velocity")

    ang_vel_ax.set_xlabel("Time (s)")

    for index, (expected, recorded, measured) in enumerate(
        zip(expected_trajectories, recorded_trajectories, measured_trajectories)
    ):
        if expected is None or recorded is None or measured is None:
            print(f"Missing trajectory for sequence {index}")
            continue

        start_time = recorded.header.stamp.to_sec()
        expected_poses = []
        expected_twists = []
        for trajectory in expected:
            if trajectory.header.stamp.to_sec() < start_time:
                continue
            expected_poses.extend(trajectory.poses)
            expected_twists.extend(trajectory.twists)
            expected_poses.append(None)
            expected_twists.append(None)
        plot_x_position(x_pos_ax, expected_poses, start_time, "Expected", line="--", marker=".")
        plot_y_position(y_pos_ax, expected_poses, start_time, "Expected", line="--", marker=".")
        plot_yaw_angle(yaw_angle_ax, expected_poses, start_time, "Expected", line="--", marker=".")
        plot_x_velocity(x_vel_ax, expected_twists, start_time, "Expected", line="--", marker=".")
        plot_ang_velocity(ang_vel_ax, expected_twists, start_time, "Expected", line="--", marker=".")

        recorded_poses = recorded.poses
        measured_poses = measured.poses

        plot_x_position(x_pos_ax, recorded_poses, start_time, "Recorded")
        plot_x_position(x_pos_ax, measured_poses, start_time, "Measured")
        plot_y_position(y_pos_ax, recorded_poses, start_time, "Recorded")
        plot_y_position(y_pos_ax, measured_poses, start_time, "Measured")
        plot_yaw_angle(yaw_angle_ax, recorded_poses, start_time, "Recorded")
        plot_yaw_angle(yaw_angle_ax, measured_poses, start_time, "Measured")

        recorded_twists = recorded.twists
        measured_twists = recorded.twists

        plot_x_velocity(x_vel_ax, recorded_twists, start_time, "Recorded")
        plot_x_velocity(x_vel_ax, measured_twists, start_time, "Measured")
        plot_ang_velocity(ang_vel_ax, recorded_twists, start_time, "Recorded")
        plot_ang_velocity(ang_vel_ax, measured_twists, start_time, "Measured")

        plot_x_velocity(x_vel_ax, commands, start_time, "Commanded")
        plot_ang_velocity(ang_vel_ax, commands, start_time, "Commanded")

    x_vel_ax.set_ylim(-2, 2)
    ang_vel_ax.set_ylim(-4, 4)

    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
