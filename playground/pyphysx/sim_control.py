import time
from dataclasses import dataclass, field

import numpy as np
import rospy
from bw_interfaces.msg import EstimatedObjectArray
from bw_shared.enums.label import FRIENDLY_ROBOT_GROUP
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.pose2d_stamped import Pose2DStamped
from bw_shared.messages.header import Header
from bw_shared.simulation_control.configs.objective_config import ObjectiveConfig
from bw_shared.simulation_control.configs.scenario_init_config import ScenarioInitConfig
from bw_shared.simulation_control.configs.sequence_element_config import SequenceElementConfig
from bw_shared.simulation_control.enums.actor_model import ActorModel
from bw_shared.simulation_control.enums.actor_role import ActorRole
from bw_shared.simulation_control.enums.objective_name import ObjectiveName
from bw_shared.simulation_control.enums.objective_type import ObjectiveType
from bw_shared.simulation_control.enums.scenario_init_type import ScenarioInitType
from bw_shared.simulation_control.enums.scenario_name import ScenarioName
from bw_shared.simulation_control.simulation_config_generator import SimulationConfigGenerator
from bw_shared.simulation_control.simulation_controller import make_simulation_controller
from matplotlib import pyplot as plt
from perception_tools.initialize_logger import initialize
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.wait_for_ros_connection import wait_for_ros_connection


@dataclass
class AppData:
    poses: list[Pose2DStamped] = field(default_factory=list)
    sim_start_time: float = 0.0


def process_robots(msg: EstimatedObjectArray, app_data: AppData) -> None:
    for robot in msg.robots:
        if robot.label not in FRIENDLY_ROBOT_GROUP:
            continue
        stamp = robot.header.stamp.to_sec()
        if stamp < app_data.sim_start_time:
            continue
        pose = Pose2D.from_msg(robot.pose.pose)
        app_data.poses.append(Pose2DStamped(header=Header.auto(stamp=stamp - app_data.sim_start_time), pose=pose))


def main() -> None:
    duration = 3.0
    time_scale = 20.0

    initialize()
    uri = wait_for_ros_connection()
    rospy.init_node("sim_control", log_level=rospy.DEBUG, disable_signals=True)
    print(f"Connected to ROS at {uri}")
    rospy.sleep(2.0)
    simulation_controller = make_simulation_controller()

    app_data = AppData()
    sim_config_generator = SimulationConfigGenerator()
    sim_config_generator.register_actor(
        ActorRole.MAIN_BOT, ActorModel.MRS_BUFF_MK2, ObjectiveName.MAIN_BOT_SIMPLE_SEQUENCE
    )
    simple_sequence = ObjectiveConfig(
        ObjectiveName.MAIN_BOT_SIMPLE_SEQUENCE,
        type=ObjectiveType.FOLLOW,
        init=ScenarioInitConfig(
            type=ScenarioInitType.RELATIVE,
            x=0.0,
            y=0.0,
            yaw=0.0,
        ),
        sequence=[
            SequenceElementConfig(timestamp=1.0, x=1.0, vx=10.0),
            SequenceElementConfig(timestamp=2.0, vx=0.0),
        ],
    )
    sim_config_generator.register_objectives(simple_sequence)
    simulation_config = sim_config_generator.generate(
        ScenarioName.SIMPLE_SCENARIO, [ActorRole.MAIN_BOT], time_scale=time_scale
    )

    ground_truth_subscriber = RosPollSubscriber("/ground_truth/robots", EstimatedObjectArray, queue_size=50)
    for _ in range(10):
        t0 = time.perf_counter()
        simulation_controller.configure_simulation(simulation_config)
        app_data.sim_start_time = time.time()
        t1 = time.perf_counter()

        for timer in simulation_controller.wait_for_timer(duration / time_scale):
            msgs = ground_truth_subscriber.receive_all()
            for msg in msgs:
                process_robots(msg, app_data)
        t2 = time.perf_counter()
        print(f"Setup took {t1 - t0:.2f} seconds")
        print(f"Simulation took {t2 - t1:.2f} seconds, received {len(app_data.poses)} poses")
    ground_truth_subscriber.unregister()

    print(f"Simulation finished. {len(app_data.poses)} poses received.")
    times = np.array([pose.header.stamp for pose in app_data.poses])
    times = times - times[0]
    xs = np.array([pose.pose.x for pose in app_data.poses])
    # ys = np.array([pose.pose.y for pose in app_data.poses])
    thetas = np.array([pose.pose.theta for pose in app_data.poses])

    fig, subplots = plt.subplots(2, 1, sharex=True)
    subplots[0].plot(times, xs, ".")
    subplots[0].set_title("X position of the robot")
    subplots[0].set_ylabel("X position (m)")
    subplots[0].set_xlabel("Time (s)")
    subplots[0].grid()
    subplots[1].plot(times, thetas, ".", label="Angle")
    subplots[1].set_title("Theta position of the robot")
    subplots[1].set_ylabel("Theta position (rad)")
    subplots[1].set_xlabel("Time (s)")
    subplots[1].grid()

    plt.show()


if __name__ == "__main__":
    main()
