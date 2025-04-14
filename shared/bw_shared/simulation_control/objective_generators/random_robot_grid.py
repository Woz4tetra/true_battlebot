import random

import numpy as np
from bw_interfaces.msg import ConfigureSimulation

from bw_shared.enums.cage_model import CageModel
from bw_shared.simulation_control.configs.fixtures_config import FixturesConfig
from bw_shared.simulation_control.configs.objective_config import ObjectiveConfig
from bw_shared.simulation_control.configs.scenario_init_config import ScenarioInitConfig
from bw_shared.simulation_control.configs.sequence_element_config import SequenceElementConfig
from bw_shared.simulation_control.configs.spotlight_config import SpotlightConfig
from bw_shared.simulation_control.enums.actor_model import ActorModel
from bw_shared.simulation_control.enums.actor_role import ActorRole
from bw_shared.simulation_control.enums.objective_name import ObjectiveName
from bw_shared.simulation_control.enums.objective_type import ObjectiveType
from bw_shared.simulation_control.enums.scenario_init_type import ScenarioInitType
from bw_shared.simulation_control.enums.scenario_name import ScenarioName
from bw_shared.simulation_control.enums.simulation_scene import SimulationScene
from bw_shared.simulation_control.enums.simulation_skyimage import SimulationSkyImage
from bw_shared.simulation_control.objective_generators.random_camera_objective import get_random_camera_objective
from bw_shared.simulation_control.simulation_config_generator import SimulationConfigGenerator


def generate_spawn_grid() -> list:
    x = np.linspace(-0.8, 0.8, 10)
    y = np.linspace(-0.8, 0.8, 10)
    grid = np.meshgrid(x, y)
    return np.array(grid).reshape(2, -1).T.tolist()


def select_from_grid(spawn_grid: list) -> tuple:
    return spawn_grid.pop(random.randint(0, len(spawn_grid) - 1))


def generate_angle() -> float:
    return random.uniform(0, 360)


def generate_target_objective(
    objective_name: ObjectiveName, spawn_grid: list, target_name: ActorRole
) -> ObjectiveConfig:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    return ObjectiveConfig(
        objective_name,
        type=ObjectiveType.TARGET,
        init=ScenarioInitConfig(type=ScenarioInitType.RELATIVE, x=x, y=y, yaw=angle),
        sequence=[SequenceElementConfig(target_name=target_name)],
    )


def generate_random_sequence(objective_name: ObjectiveName, spawn_grid: list, duration: float) -> ObjectiveConfig:
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
            SequenceElementConfig(
                timestamp=timestamp,
                x=x,
                y=y,
                yaw=angle,
                vx=10.0,
                vyaw=720.0,
            )
        )
    return ObjectiveConfig(
        objective_name,
        type=ObjectiveType.FOLLOW,
        init=ScenarioInitConfig(
            type=ScenarioInitType.RELATIVE,
            x=x,
            y=y,
            yaw=angle,
            z=0.0 if right_side_up else 0.1,
            roll=0.0 if right_side_up else 180.0,
        ),
        sequence=sequence,
    )


def generate_random_idle(objective_name: ObjectiveName, spawn_grid: list) -> ObjectiveConfig:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    return ObjectiveConfig(
        objective_name,
        type=ObjectiveType.IDLE,
        init=ScenarioInitConfig(type=ScenarioInitType.RELATIVE, x=x, y=y, yaw=angle),
    )


def generate_random_robot_grid_scenario(
    scenario_name: ScenarioName,
    generator: SimulationConfigGenerator,
    selected_cages: list[CageModel],
) -> ConfigureSimulation:
    change_cage = random.uniform(0, 1) > 0.7
    change_background = random.uniform(0, 1) > 0.7

    if change_cage:
        generator.set_cage_size(random.choice(selected_cages))
        generator.background.sky_image = random.choice(list(SimulationSkyImage))

    if change_background:
        generator.background.name = random.choice(list(SimulationScene))

    if generator.cage.cage_type == CageModel.NHRL_3LB_CAGE:
        spawn_referee = random.uniform(0, 1) > 0.5
    else:
        spawn_referee = False

    generator.stretch_cage_size(dx=random.uniform(-0.1, 0.1), dy=random.uniform(-0.02, 0.02))

    num_robots = random.randint(1, 3)

    generator.fixtures = FixturesConfig(
        spotlight=SpotlightConfig(
            enabled=True,
            range=random.uniform(2.5, 3.5),
            spot_angle=random.uniform(105, 120),
            intensity=random.uniform(5.0, 7.0),
            shadow_strength=random.uniform(0.6, 0.8),
        )
    )

    mini_bot_model = ActorModel.MR_STABS_MK2 if random.uniform(0, 1) < 0.7 else ActorModel.MR_STABS_A_02
    main_bot_model = ActorModel.MRS_BUFF_MK2 if random.uniform(0, 1) < 0.7 else ActorModel.MRS_BUFF_MK1

    mini_bot_objective = (
        ObjectiveName.RANDOMIZED_START_TARGET_OPPONENT
        if num_robots >= 3
        else ObjectiveName.MINI_BOT_RANDOMIZED_SEQUENCE
    )
    generator.register_actor(ActorRole.MINI_BOT, mini_bot_model, mini_bot_objective)
    generator.register_actor(ActorRole.MAIN_BOT, main_bot_model, ObjectiveName.MAIN_BOT_RANDOMIZED_SEQUENCE)
    generator.register_actor(
        ActorRole.OPPONENT_1, ActorModel.MRS_BUFF_MK1_BIZARRO, ObjectiveName.RANDOMIZED_START_TARGET_MAIN
    )
    generator.register_actor(ActorRole.REFEREE, ActorModel.REFEREE, ObjectiveName.RANDOMIZED_IDLE)
    generator.register_actor(ActorRole.CAMERA_1, ActorModel.TRAINING_CAMERA, ObjectiveName.RANDOMIZED_CAMERA)

    actor_roles = [ActorRole.MINI_BOT, ActorRole.MAIN_BOT, ActorRole.OPPONENT_1]
    actor_roles = actor_roles[:num_robots]

    actor_roles.append(ActorRole.CAMERA_1)
    if spawn_referee:
        actor_roles.append(ActorRole.REFEREE)

    generator.cage.display_readout = False
    return generator.generate(scenario_name, actor_roles)


def generate_random_robot_grid(
    scenario_name: ScenarioName, generator: SimulationConfigGenerator, selected_cages: list[CageModel], duration: float
) -> ConfigureSimulation:
    # generate spawn grid
    spawn_grid = generate_spawn_grid()

    # generate camera objective
    camera_objective = get_random_camera_objective(ObjectiveName.RANDOMIZED_CAMERA, generator.cage.cage_type)

    # generate random sequence for mini_bot
    mini_bot_random_sequence = generate_random_sequence(
        ObjectiveName.MINI_BOT_RANDOMIZED_SEQUENCE, spawn_grid, duration
    )

    # generate target sequence for mini_bot
    mini_bot_target_sequence = generate_target_objective(
        ObjectiveName.RANDOMIZED_START_TARGET_OPPONENT, spawn_grid, ActorRole.OPPONENT_1
    )

    # generate random sequence for main_bot
    main_bot_random_sequence = generate_random_sequence(
        ObjectiveName.MAIN_BOT_RANDOMIZED_SEQUENCE, spawn_grid, duration
    )

    # generate target sequence for opponent_1
    opponent_1_target_sequence = generate_target_objective(
        ObjectiveName.RANDOMIZED_START_TARGET_MAIN, spawn_grid, ActorRole.MAIN_BOT
    )

    # generate idle sequence for referee
    referee_idle_sequence = generate_random_idle(ObjectiveName.RANDOMIZED_IDLE, spawn_grid)

    generator.register_objectives(
        camera_objective,
        mini_bot_random_sequence,
        mini_bot_target_sequence,
        main_bot_random_sequence,
        opponent_1_target_sequence,
        referee_idle_sequence,
    )

    return generate_random_robot_grid_scenario(scenario_name, generator, selected_cages)
