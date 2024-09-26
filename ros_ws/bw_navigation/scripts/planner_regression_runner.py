from dataclasses import dataclass
from threading import Event

import numpy as np
import rospy
from actionlib import SimpleActionClient
from bw_interfaces.msg import ConfigureSimulation, GoToGoalAction, GoToGoalFeedback, GoToGoalGoal
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.pid.config import PidConfig
from bw_shared.simulation_control.make_objective import make_objective
from bw_shared.simulation_control.simulation_controller import SimulationController, make_simulation_controller
from bw_tools.messages.goal_strategy import GoalStrategy
from bw_tools.messages.goal_type import GoalType
from geometry_msgs.msg import PoseStamped


@dataclass
class AppData:
    total_time_set: Event
    total_time: float = 0.0


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


def configure_simulation(simulation_controller: SimulationController, run_config: RunConfig) -> None:
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
    if not app.total_time_set.is_set():
        rospy.logerr("Timed out waiting for total time")
        return False

    wait_time = app.total_time + 3.0

    for timer in simulation_controller.wait_for_timer(wait_time):
        if rospy.is_shutdown():
            break
    return True


def main() -> None:
    rospy.init_node("planner_regression_runner")

    app = AppData(total_time_set=Event())

    simulation_controller = make_simulation_controller()

    go_to_goal_client = SimpleActionClient("go_to_goal", GoToGoalAction)
    rospy.loginfo("Waiting for go to goal action server")
    go_to_goal_client.wait_for_server()
    rospy.loginfo("Go to goal action server is ready")

    run_config = RunConfig(
        initial_pose=Pose2D(x=-1.0, y=0.0, theta=0.0),
        goal_pose=Pose2D(x=1.0, y=0.0, theta=0.0),
    )
    configure_simulation(simulation_controller, run_config)
    send_action_goal(app, go_to_goal_client, run_config)
    wait_for_trajectory(app, simulation_controller)


if __name__ == "__main__":
    main()
