import random
from dataclasses import dataclass

import numpy as np
from bw_interfaces.msg import ConfigureSimulation, SimulationConfig

from bw_shared.configs.size import Size
from bw_shared.enums.cage_model import CageModel
from bw_shared.enums.simulation_scene import SimulationScene
from bw_shared.enums.simulation_skyimage import SimulationSkyImage
from bw_shared.simulation_control.make_objective import make_objective
from bw_shared.simulation_control.randomized.random_camera_objective import get_random_camera_objective


@dataclass
class SceneSession:
    last_cage: CageModel = CageModel.NHRL_3LB_CAGE
    last_background: SimulationScene = SimulationScene.GARAGE_SCENE
    last_skyimage: SimulationSkyImage = SimulationSkyImage.HAVOC1


def generate_spawn_grid() -> list:
    x = np.linspace(-0.8, 0.8, 10)
    y = np.linspace(-0.8, 0.8, 10)
    grid = np.meshgrid(x, y)
    return np.array(grid).reshape(2, -1).T.tolist()


def select_from_grid(spawn_grid: list) -> tuple:
    return spawn_grid.pop(random.randint(0, len(spawn_grid) - 1))


def generate_angle() -> float:
    return random.uniform(0, 360)


def generate_target_objective(objective_name: str, spawn_grid: list, target_name: str) -> SimulationConfig:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    return make_objective(
        objective_name,
        {
            "type": "target",
            "init": {"type": "relative", "x": x, "y": y, "yaw": angle},
            "sequence": [{"target_name": target_name}],
        },
    )


def generate_random_sequence(objective_name: str, spawn_grid: list, duration: float) -> SimulationConfig:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    timestamps = np.arange(0, duration, 0.5)
    right_side_up = random.choice([True, False])
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
    return make_objective(
        objective_name,
        {
            "type": "follow",
            "init": {
                "type": "relative",
                "x": x,
                "y": y,
                "yaw": angle,
                "z": 0.0 if right_side_up else 0.1,
                "roll": 0.0 if right_side_up else 180.0,
            },
            "sequence": sequence,
        },
    )


def generate_idle(objective_name: str, spawn_grid: list) -> SimulationConfig:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    return make_objective(
        objective_name,
        {
            "type": "idle",
            "init": {"type": "relative", "x": x, "y": y, "yaw": angle},
            "sequence": [],
        },
    )


def generate_random_robot_grid_scenario(
    scenario_name: str, scene_session: SceneSession, cage_sizes: dict[CageModel, Size]
) -> SimulationConfig:
    change_cage = random.uniform(0, 1) > 0.7
    change_background = random.uniform(0, 1) > 0.7

    if change_cage:
        scene_session.last_cage = random.choice(list(cage_sizes.keys()))
        scene_session.last_skyimage = random.choice(list(SimulationSkyImage))

    if change_background:
        scene_session.last_background = random.choice(list(SimulationScene))

    if scene_session.last_cage == CageModel.NHRL_3LB_CAGE:
        spawn_referee = random.uniform(0, 1) > 0.5
    else:
        spawn_referee = False

    cage_size = cage_sizes[scene_session.last_cage]
    cage_x = cage_size.x + random.uniform(-0.1, 0.1)
    cage_y = cage_x + random.uniform(-0.02, 0.02)

    num_robots = random.randint(1, 3)

    fixtures_config = {
        "spotlight": {
            "enabled": True,
            "range": random.uniform(2.5, 3.5),
            "spot_angle": random.uniform(105, 120),
            "intensity": random.uniform(5.0, 7.0),
            "shadow_strength": random.uniform(0.6, 0.8),
        }
    }

    mini_bot_model = "MR STABS MK2" if random.uniform(0, 1) < 0.7 else "MR STABS A-02"
    main_bot_model = "MRS BUFF MK2" if random.uniform(0, 1) < 0.7 else "MRS BUFF B-03"

    mini_bot_objective = "randomized_start_target_opponent" if num_robots >= 3 else "mini_bot_randomized_sequence"
    mr_stabs_mk2 = {
        "name": "mini_bot",
        "model": mini_bot_model,
        "objective": mini_bot_objective,
    }
    mrs_buff_mk2 = {
        "name": "main_bot",
        "model": main_bot_model,
        "objective": "main_bot_randomized_sequence",
    }
    opponent_1 = {
        "name": "opponent_1",
        "model": "MRS BUFF B-03 Bizarro",
        "objective": "randomized_start_target_main",
    }
    robot_actors = [mr_stabs_mk2, mrs_buff_mk2, opponent_1]
    robot_actors = robot_actors[:num_robots]

    actors = [
        {"name": "tracking_camera", "model": "Training Camera", "objective": "randomized_camera"},
    ] + robot_actors
    if spawn_referee:
        actors.append({"name": "referee", "model": "Referee", "objective": "randomized_idle"})
    scenario_data = {
        "cage": {
            "dims": {"x": cage_x, "y": cage_y},
            "cage_type": scene_session.last_cage.value,
            "display_readout": False,
        },
        "fixtures": fixtures_config,
        "background": {"name": scene_session.last_background.value, "sky_image": scene_session.last_skyimage.value},
        "actors": actors,
    }
    return make_objective(scenario_name, scenario_data)


def generate_random_robot_grid(
    scenario_name: str, scene_session: SceneSession, cage_sizes: dict[CageModel, Size], duration: float
) -> ConfigureSimulation:
    # generate spawn grid
    spawn_grid = generate_spawn_grid()

    # generate camera objective
    camera_objective = get_random_camera_objective("randomized_camera", scene_session.last_cage)

    # generate random sequence for mini_bot
    mini_bot_random_sequence = generate_random_sequence("mini_bot_randomized_sequence", spawn_grid, duration)

    # generate target sequence for mini_bot
    mini_bot_target_sequence = generate_target_objective("randomized_start_target_opponent", spawn_grid, "opponent_1")

    # generate random sequence for main_bot
    main_bot_random_sequence = generate_random_sequence("main_bot_randomized_sequence", spawn_grid, duration)

    # generate target sequence for opponent_1
    opponent_1_target_sequence = generate_target_objective("randomized_start_target_main", spawn_grid, "main_bot")

    # generate idle sequence for referee
    referee_idle_sequence = generate_idle("randomized_idle", spawn_grid)

    # create scenario
    return ConfigureSimulation(
        scenario=generate_random_robot_grid_scenario(scenario_name, scene_session, cage_sizes),
        objectives=[
            camera_objective,
            mini_bot_random_sequence,
            mini_bot_target_sequence,
            main_bot_random_sequence,
            opponent_1_target_sequence,
            referee_idle_sequence,
        ],
    )
