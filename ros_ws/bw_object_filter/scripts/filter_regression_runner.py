import argparse
from dataclasses import dataclass, field
from datetime import datetime
from threading import Lock

import numpy as np
import rospy
from bw_interfaces.msg import CageCorner as RosCageCorner
from bw_interfaces.msg import ConfigureSimulation, Trajectory
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.pid.config import PidConfig
from bw_shared.simulation_control.make_objective import make_objective
from bw_shared.simulation_control.simulation_controller import SimulationController, make_simulation_controller
from bw_tools.messages.cage_corner import CageCorner
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rosbag.bag import Bag
from std_msgs.msg import Empty


@dataclass
class AppData:
    bag: Bag
    recorded_trajectory: Trajectory = field(default_factory=lambda: Trajectory())
    measured_trajectory: Trajectory = field(default_factory=lambda: Trajectory())
    lock: Lock = field(default_factory=Lock)
    sequence: int = 0


@dataclass
class RunConfig:
    initial_pose: Pose2D
    linear_pid: PidConfig = PidConfig(kp=0.0, ki=0.0, kd=0.0, kf=1.0)
    angular_pid: PidConfig = PidConfig(kp=0.0, ki=0.0, kd=0.0, kf=1.0)


def configure_simulation(
    simulation_controller: SimulationController, cage_corner_pub: rospy.Publisher, run_config: RunConfig
) -> None:
    auto_objective = make_objective(
        "mini_bot_auto",
        {
            "type": "auto",
            "init": {
                "type": "absolute",
                "x": run_config.initial_pose.x,
                "y": run_config.initial_pose.y,
                "yaw": np.rad2deg(run_config.initial_pose.theta),
            },
            "linear_pid": {
                "kp": run_config.linear_pid.kp,
                "ki": run_config.linear_pid.ki,
                "kd": run_config.linear_pid.kd,
                "kf": run_config.linear_pid.kf,
            },
            "angular_pid": {
                "kp": run_config.angular_pid.kp,
                "ki": run_config.angular_pid.ki,
                "kd": run_config.angular_pid.kd,
                "kf": run_config.angular_pid.kf,
            },
            "sequence": [],
        },
    )
    scenario = make_objective(
        "mini_bot_follow_path",
        {
            "cage": {"dims": {"x": 2.35, "y": 2.35}},
            "background": {"name": "Panorama Scene", "sky_image": "havoc1"},
            "actors": [
                {"name": "slow_camera", "model": "ZED 2i", "objective": "blue_slow_camera"},
                {"name": "tracking_camera", "model": "OAK-1", "objective": "blue_tracking_camera"},
                {"name": "mini_bot", "model": "MR STABS MK2", "objective": "mini_bot_auto"},
            ],
        },
    )

    simulation_config = ConfigureSimulation(
        scenario=scenario,
        objectives=[auto_objective],
    )
    simulation_controller.configure_simulation(simulation_config)

    cage_corner_pub.publish(CageCorner.BLUE_SIDE.to_msg())


def wait_for_poses_to_settle(app: AppData, run_config: RunConfig) -> None:
    rospy.loginfo("Waiting for poses to settle")
    while not rospy.is_shutdown():
        measured_trajectory = app.recorded_trajectory
        if len(measured_trajectory.poses) == 0:
            continue
        last_pose = measured_trajectory.poses[-1]
        if (
            np.abs(last_pose.pose.position.x - run_config.initial_pose.x) < 0.01
            and np.abs(last_pose.pose.position.y - run_config.initial_pose.y) < 0.01
        ):
            break

        rospy.sleep(0.1)


def run_command(
    app: AppData,
    simulation_controller: SimulationController,
    command_pub: rospy.Publisher,
    linear_x: float,
    angular: float,
    duration: float,
) -> bool:
    command = Twist()
    command.linear.x = linear_x
    command.angular.z = angular
    twists = []
    for timer in simulation_controller.wait_for_timer(duration):
        command_pub.publish(command)
        twists.append(TwistStamped(header=rospy.Header(stamp=rospy.Time.now()), twist=command))
        if rospy.is_shutdown():
            break
    with app.lock:
        app.recorded_trajectory.header.seq = app.sequence
        app.measured_trajectory.header.seq = app.sequence
        app.sequence += 1
        app.bag.write("recorded_trajectory", app.recorded_trajectory)
        app.bag.write("measured_trajectory", app.measured_trajectory)
        command_trajectory = Trajectory()
        command_trajectory.header.seq = app.sequence
        command_trajectory.twists = twists
        app.bag.write("commands", command_trajectory)
    return True


def append_odom_to_trajectory(app: AppData, trajectory: Trajectory, msg: Odometry) -> None:
    with app.lock:
        if trajectory.header.stamp == rospy.Time(0):
            trajectory.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        twist = TwistStamped()
        twist.header = msg.header
        twist.twist = msg.twist.twist

        trajectory.poses.append(pose)
        trajectory.twists.append(twist)


def ground_truth_callback(app: AppData, msg: Odometry) -> None:
    append_odom_to_trajectory(app, app.recorded_trajectory, msg)


def measured_callback(app: AppData, msg: Odometry) -> None:
    append_odom_to_trajectory(app, app.measured_trajectory, msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run filter regression test")
    parser.add_argument("--bag", type=str, help="Path to bag directory")
    args = parser.parse_args()

    rospy.init_node("filter_regression_runner")

    bag_dir = args.bag if args.bag else "/data/bags"
    bag_path = f"{bag_dir}/filter_regression_{datetime.now().strftime('%Y-%m-%dT%H-%M-%S')}.bag"
    rospy.loginfo(f"Recording to {bag_path}")
    bag = Bag(bag_path, "w")
    app = AppData(bag=bag)

    cage_corner_pub = rospy.Publisher("set_cage_corner", RosCageCorner, queue_size=1, latch=True)
    simulation_controller = make_simulation_controller()

    rospy.Subscriber(
        "/mini_bot/ground_truth",
        Odometry,
        lambda msg: ground_truth_callback(app, msg),
        queue_size=1,
    )

    rospy.Subscriber(
        "/mini_bot/odom",
        Odometry,
        lambda msg: measured_callback(app, msg),
        queue_size=1,
    )

    command_pub = rospy.Publisher(
        "/mini_bot/cmd_vel",
        Twist,
        queue_size=1,
    )

    reset_filters_pub = rospy.Publisher(
        "/mini_bot/reset_filters",
        Empty,
        queue_size=1,
    )

    commands = [
        (0.5, 0.0, 2.0, -1.1),
        (1.0, 0.0, 2.0, -1.1),
        (3.0, 0.0, 2.0, -1.1),
        (6.0, 0.0, 2.0, -1.1),
        (0.0, 10.0, 2.0, 0.0),
        (0.0, 30.0, 2.0, 0.0),
        (0.0, 50.0, 2.0, 0.0),
        (0.5, 5.0, 2.0, 0.0),
        (1.0, 10.0, 2.0, 0.0),
        (3.0, 30.0, 2.0, 0.0),
        (6.0, 50.0, 2.0, 0.0),
    ]
    try:
        for linear_x, angular_z, duration, start_dist in commands:
            reset_filters_pub.publish(Empty())
            rospy.sleep(1.0)
            run_config = RunConfig(
                initial_pose=Pose2D(x=start_dist, y=start_dist, theta=np.pi / 4),
            )
            configure_simulation(simulation_controller, cage_corner_pub, run_config)
            wait_for_poses_to_settle(app, run_config)
            run_command(app, simulation_controller, command_pub, linear_x, angular_z, duration)
    finally:
        bag.close()


if __name__ == "__main__":
    main()
