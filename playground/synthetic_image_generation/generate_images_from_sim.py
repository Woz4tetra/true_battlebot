import json
import random
import time

import numpy as np
import rospy
from bw_interfaces.msg import ConfigureSimulation, SimulationConfig
from bw_shared.geometry.projection_math.look_rotation import look_rotation
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import String


def compute_camera_pose(
    distance: float, azimuth_angle: float, elevation_angle: float
) -> Transform3D:
    position_array = np.array(
        [
            distance * np.cos(elevation_angle) * np.cos(azimuth_angle),
            -1 * distance * np.cos(elevation_angle) * np.sin(azimuth_angle),
            distance * np.sin(elevation_angle),
        ]
    )

    rotation = look_rotation(-1 * position_array)

    return Transform3D.from_position_and_quaternion(Vector3(*position_array), rotation)


def get_random_camera_pose() -> Transform3D:
    angle_inset = 0.2
    distance_range = (2.2, 2.5)
    azimuth_angle_range = (-np.pi / 4 + angle_inset, np.pi / 4 - angle_inset)
    elevation_angle_range = (0.4, 0.5)
    flip_azimuth = random.uniform(0.0, 1.0) > 0.5

    distance = random.uniform(*distance_range)
    azimuth_angle = random.uniform(*azimuth_angle_range)
    elevation_angle = random.uniform(*elevation_angle_range)

    if flip_azimuth:
        azimuth_angle = azimuth_angle + np.pi

    pose = compute_camera_pose(distance, azimuth_angle, elevation_angle)
    rotation = pose.quaternion_np
    randomized_rotation = np.random.normal(rotation, 0.01)
    camera_pose = Transform3D.from_position_and_quaternion(
        pose.position, Quaternion(*randomized_rotation)
    )
    return camera_pose


def get_random_camera_objective() -> dict:
    camera_pose = get_random_camera_pose()

    rpy_deg = np.degrees(camera_pose.rpy)

    return {
        "type": "idle",
        "init": {
            "type": "flu",
            "x": camera_pose.position.x,
            "y": camera_pose.position.y,
            "z": camera_pose.position.z,
            "roll": rpy_deg[0],
            "pitch": rpy_deg[1],
            "yaw": rpy_deg[2],
        },
        "sequence": [],
    }


def generate_spawn_grid() -> list:
    x = np.linspace(-0.8, 0.8, 10)
    y = np.linspace(-0.8, 0.8, 10)
    grid = np.meshgrid(x, y)
    return np.array(grid).reshape(2, -1).T.tolist()


def select_from_grid(spawn_grid: list) -> tuple:
    return spawn_grid.pop(random.randint(0, len(spawn_grid) - 1))


def generate_angle() -> float:
    return random.uniform(0, 360)


def generate_target_objective(spawn_grid: list, target_name: str) -> dict:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    return {
        "type": "target",
        "init": {"type": "relative", "x": x, "y": y, "yaw": angle},
        "sequence": [{"target_name": target_name}],
    }


def generate_random_sequence(spawn_grid: list, duration: float) -> dict:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    timestamps = np.arange(0, duration, 0.5)
    sequence = []
    for timestamp in timestamps:
        angle = generate_angle()
        x = random.uniform(-0.8, 0.8)
        y = random.uniform(-0.8, 0.8)
        sequence.append(
            {
                "timestamp": timestamp,
                "x": x,
                "y": y,
                "yaw": angle,
                "vx": 10.0,
                "vyaw": 720.0,
            }
        )
    return {
        "type": "follow",
        "init": {
            "type": "relative",
            "x": x,
            "y": y,
            "yaw": angle,
        },
        "sequence": sequence,
    }


def generate_idle(spawn_grid: list) -> dict:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    return {
        "type": "idle",
        "init": {"type": "relative", "x": x, "y": y, "yaw": angle},
        "sequence": [],
    }


def generate_scenario(num_bots: int) -> dict:
    cage_x = random.uniform(2.2, 2.4)
    cage_y = cage_x + random.uniform(-0.02, 0.02)
    mini_bot_objective = (
        "randomized_start_target_opponent"
        if num_bots >= 3
        else "mini_bot_randomized_sequence"
    )
    mini_bot = {
        "name": "mini_bot",
        "model": "MR STABS MK2",
        "objective": mini_bot_objective,
    }
    main_bot = {
        "name": "main_bot",
        "model": "MRS BUFF MK2",
        "objective": "main_bot_randomized_sequence",
    }
    opponent_1 = {
        "name": "opponent_1",
        "model": "MRS BUFF B-03 Bizarro",
        "objective": "randomized_start_target_main",
    }
    robot_actors = [mini_bot, main_bot, opponent_1]
    robot_actors = robot_actors[:num_bots]
    actors = [
        {"name": "tracking_camera", "model": "OAK-1", "objective": "randomized_camera"},
        {"name": "referee", "model": "Referee", "objective": "randomized_idle"},
    ] + robot_actors
    return {
        "cage": {"dims": {"x": cage_x, "y": cage_y}},
        "actors": actors,
    }


def main() -> None:
    rospy.init_node("generate_images_from_sim")

    configure_simulation_pub = rospy.Publisher(
        "/simulation/add_configuration", ConfigureSimulation, queue_size=1
    )
    select_scenario_pub = rospy.Publisher(
        "/simulation/scenario_selection", String, queue_size=1
    )

    rospy.sleep(2.0)  # wait for publishers to connect

    scenario_name = "image_synthesis"

    while not rospy.is_shutdown():
        # select number of robots and duration. Set camera pose.
        num_bots = random.randint(1, 3)
        duration = random.uniform(5, 10)

        # generate spawn grid
        spawn_grid = generate_spawn_grid()

        # generate camera objective
        camera_objective = SimulationConfig(
            name="randomized_camera",
            json_data=json.dumps(get_random_camera_objective()),
        )

        # generate random sequence for mini_bot
        mini_bot_random_sequence = SimulationConfig(
            name="mini_bot_randomized_sequence",
            json_data=json.dumps(generate_random_sequence(spawn_grid, duration)),
        )

        # generate target sequence for mini_bot
        mini_bot_target_sequence = SimulationConfig(
            name="randomized_start_target_opponent",
            json_data=json.dumps(generate_target_objective(spawn_grid, "opponent_1")),
        )

        # generate random sequence for main_bot
        main_bot_random_sequence = SimulationConfig(
            name="main_bot_randomized_sequence",
            json_data=json.dumps(generate_random_sequence(spawn_grid, duration)),
        )

        # generate target sequence for opponent_1
        opponent_1_target_sequence = SimulationConfig(
            name="randomized_start_target_main",
            json_data=json.dumps(generate_target_objective(spawn_grid, "main_bot")),
        )

        # generate idle sequence for referee
        referee_idle_sequence = SimulationConfig(
            name="randomized_idle",
            json_data=json.dumps(generate_idle(spawn_grid)),
        )

        # create scenario
        scenario = generate_scenario(num_bots)
        simulation_config = ConfigureSimulation(
            scenario=SimulationConfig(
                name=scenario_name,
                json_data=json.dumps(scenario),
            ),
            objectives=[
                camera_objective,
                mini_bot_random_sequence,
                mini_bot_target_sequence,
                main_bot_random_sequence,
                opponent_1_target_sequence,
                referee_idle_sequence,
            ],
        )

        # publish scenario and objectives. Start scenario.
        print(f"Starting scenario with {num_bots} robots for {duration} seconds.")
        configure_simulation_pub.publish(simulation_config)
        rospy.sleep(0.2)  # wait config to process
        select_scenario_pub.publish(scenario_name)

        # wait for scenario to finish. Repeat.
        start_time = time.monotonic()
        while time.monotonic() - start_time < duration:
            if rospy.is_shutdown():
                break
            rospy.sleep(0.01)


if __name__ == "__main__":
    main()
