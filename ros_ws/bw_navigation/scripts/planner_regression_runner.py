from dataclasses import dataclass
from threading import Event

import numpy as np
import rospy
from actionlib import SimpleActionClient
from bw_interfaces.msg import ConfigureSimulation, GoToGoalAction, GoToGoalFeedback, GoToGoalGoal
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.simulation_control.make_objective import make_objective
from bw_shared.simulation_control.simulation_controller import make_simulation_controller
from bw_tools.messages.goal_strategy import GoalStrategy
from bw_tools.messages.goal_type import GoalType
from geometry_msgs.msg import PoseStamped


@dataclass
class AppData:
    total_time_set: Event
    total_time: float = 0.0


def feedback_cb(app: AppData, feedback: GoToGoalFeedback) -> None:
    if np.isnan(feedback.total_time):
        return
    app.total_time = feedback.total_time
    app.total_time_set.set()


def main() -> None:
    rospy.init_node("planner_regression_runner")

    app = AppData(total_time_set=Event())

    simulation_controller = make_simulation_controller()

    auto_objective = make_objective(
        "mini_bot_auto",
        {
            "type": "auto",
            "init": {
                "type": "flu",
                "x": 0.85,
                "y": -0.5,
                "yaw": 135.0,
            },
            "linear_pid": {
                "kp": 20.0,
                "ki": 0.1,
                "kd": 0.005,
                "kf": 1.0,
            },
            "angular_pid": {
                "kp": 20.0,
                "ki": 0.01,
                "kd": 0.001,
                "kf": 1.0,
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
                {"name": "mini_bot", "model": "MR STABS MK2", "objective": "auto_blue_corner"},
            ],
        },
    )

    simulation_config = ConfigureSimulation(
        scenario=scenario,
        objectives=[auto_objective],
    )
    simulation_controller.configure_simulation(simulation_config)

    go_to_goal_client = SimpleActionClient("go_to_goal", GoToGoalAction)
    rospy.loginfo("Waiting for go to goal action server")
    go_to_goal_client.wait_for_server()
    rospy.loginfo("Go to goal action server is ready")

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose = Pose2D(x=0.85, y=-0.5, theta=0.0).to_msg()

    goal = GoToGoalGoal()
    goal.goal = goal_pose
    goal.strategy = GoalStrategy.CRASH_TRAJECTORY_PLANNER.value
    goal.goal_type = GoalType.FIXED_POSE.value
    go_to_goal_client.send_goal(goal, feedback_cb=lambda feedback: feedback_cb(app, feedback))

    app.total_time_set.wait()

    for timer in simulation_controller.wait_for_timer(app.total_time):
        if rospy.is_shutdown():
            break


if __name__ == "__main__":
    main()
