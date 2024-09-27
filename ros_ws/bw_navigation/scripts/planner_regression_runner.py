import argparse
from dataclasses import dataclass, field
from datetime import datetime
from threading import Event, Lock

import numpy as np
import rospy
from actionlib import SimpleActionClient
from bw_interfaces.msg import CageCorner as RosCageCorner
from bw_interfaces.msg import ConfigureSimulation, GoToGoalAction, GoToGoalFeedback, GoToGoalGoal, Trajectory
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.pid.config import PidConfig
from bw_shared.simulation_control.make_objective import make_objective
from bw_shared.simulation_control.simulation_controller import SimulationController, make_simulation_controller
from bw_tools.messages.cage_corner import CageCorner
from bw_tools.messages.goal_strategy import GoalStrategy
from bw_tools.messages.goal_type import GoalType
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosbag.bag import Bag


@dataclass
class AppData:
    total_time_set: Event
    bag: Bag
    total_time: float = 0.0
    expected_trajectory: Trajectory = field(default_factory=lambda: Trajectory())
    recorded_trajectory: Trajectory = field(default_factory=lambda: Trajectory())
    lock: Lock = field(default_factory=Lock)
    sequence: int = 0


@dataclass
class RunConfig:
    initial_pose: Pose2D
    goal_pose: Pose2D
    linear_pid: PidConfig = PidConfig(kp=20.0, ki=0.1, kd=0.005, kf=1.0)
    angular_pid: PidConfig = PidConfig(kp=30.0, ki=0.01, kd=0.0001, kf=1.0)


def feedback_cb(app: AppData, feedback: GoToGoalFeedback) -> None:
    if np.isnan(feedback.total_time):
        return
    app.total_time = feedback.total_time
    app.total_time_set.set()
    app.expected_trajectory = feedback.trajectory


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


def send_action_goal(app: AppData, go_to_goal_client: SimpleActionClient, run_config: RunConfig) -> None:
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose = run_config.goal_pose.to_msg()

    goal = GoToGoalGoal()
    goal.goal = goal_pose
    goal.strategy = GoalStrategy.CRASH_TRAJECTORY_PLANNER.value
    goal.goal_type = GoalType.FIXED_POSE.value
    go_to_goal_client.send_goal(goal, feedback_cb=lambda feedback: feedback_cb(app, feedback))


def wait_for_trajectory(app: AppData, simulation_controller: SimulationController) -> bool:
    app.total_time_set.clear()
    app.total_time_set.wait(timeout=5.0)
    with app.lock:
        app.recorded_trajectory = Trajectory()
    if not app.total_time_set.is_set():
        rospy.logerr("Timed out waiting for total time")
        return False

    wait_time = app.total_time + 3.0

    for timer in simulation_controller.wait_for_timer(wait_time):
        if rospy.is_shutdown():
            break
    with app.lock:
        app.expected_trajectory.header.seq = app.sequence
        app.recorded_trajectory.header.seq = app.sequence
        app.sequence += 1
        app.bag.write("expected_trajectory", app.expected_trajectory)
        app.bag.write("recorded_trajectory", app.recorded_trajectory)
    return True


def ground_truth_callback(app: AppData, msg: Odometry) -> None:
    with app.lock:
        trajectory = app.recorded_trajectory
        if trajectory.header.stamp == rospy.Time(0):
            trajectory.header = msg.header
        trajectory.poses.append(msg.pose.pose)
        trajectory.twists.append(msg.twist.twist)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run planner regression test")
    parser.add_argument("--bag", type=str, help="Path to bag directory")
    args = parser.parse_args()

    rospy.init_node("planner_regression_runner")

    bag_dir = args.bag if args.bag else "/media/storage/bags"
    bag_path = f"{bag_dir}/planner_regression_{datetime.now().strftime('%Y-%m-%dT%H-%M-%S')}.bag"
    rospy.loginfo(f"Recording to {bag_path}")
    bag = Bag(bag_path, "w")
    app = AppData(total_time_set=Event(), bag=bag)

    cage_corner_pub = rospy.Publisher("set_cage_corner", RosCageCorner, queue_size=1, latch=True)
    simulation_controller = make_simulation_controller()

    rospy.Subscriber(
        "/mini_bot/ground_truth",
        Odometry,
        lambda msg: ground_truth_callback(app, msg),
        queue_size=1,
    )

    go_to_goal_client = SimpleActionClient("go_to_goal", GoToGoalAction)
    rospy.loginfo("Waiting for go to goal action server")
    go_to_goal_client.wait_for_server()
    rospy.loginfo("Go to goal action server is ready")

    run_config = RunConfig(
        initial_pose=Pose2D(x=-1.0, y=0.0, theta=0.0),
        goal_pose=Pose2D(x=1.0, y=0.0, theta=0.0),
    )
    try:
        configure_simulation(simulation_controller, cage_corner_pub, run_config)
        send_action_goal(app, go_to_goal_client, run_config)
        wait_for_trajectory(app, simulation_controller)
    finally:
        bag.close()


if __name__ == "__main__":
    main()
