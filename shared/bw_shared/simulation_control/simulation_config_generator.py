from typing import List, Optional

from bw_interfaces.msg import ConfigureSimulation

from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.cage_model import CageModel
from bw_shared.simulation_control.configs.actor_config import ActorConfig
from bw_shared.simulation_control.configs.background_config import BackgroundConfig
from bw_shared.simulation_control.configs.cage_config import CageConfig
from bw_shared.simulation_control.configs.camera_config import CameraConfig
from bw_shared.simulation_control.configs.dims_config import DimsConfig
from bw_shared.simulation_control.configs.fixtures_config import FixturesConfig
from bw_shared.simulation_control.configs.objective_config import ObjectiveConfig
from bw_shared.simulation_control.configs.physics_material_config import PhysicsMaterialsConfig
from bw_shared.simulation_control.configs.scenario_config import ScenarioConfig
from bw_shared.simulation_control.enums.actor_model import ActorModel
from bw_shared.simulation_control.enums.actor_role import ActorRole
from bw_shared.simulation_control.enums.objective_name import ObjectiveName
from bw_shared.simulation_control.enums.scenario_name import ScenarioName
from bw_shared.simulation_control.load_cage_model_sizes import load_cage_model_sizes


class SimulationConfigGenerator:
    def __init__(self, shared_config: Optional[SharedConfig] = None) -> None:
        if shared_config is None:
            self.shared_config = SharedConfig.from_files()
        else:
            self.shared_config = shared_config
        self.cage_sizes = load_cage_model_sizes(self.shared_config.maps)
        self.actors: dict[ActorRole, ActorConfig] = {}
        self.objectives: dict[str, ObjectiveConfig] = {}
        self.scenarios: dict[str, ScenarioConfig] = {}
        self.materials: dict[str, PhysicsMaterialsConfig] = {}
        self.cage = CageConfig()
        self.background = BackgroundConfig()
        self.spectator_camera: Optional[CameraConfig] = None
        self.fixtures = FixturesConfig()

        self.set_cage_size(self.cage.cage_type)

    def set_cage_size(self, model: CageModel) -> None:
        dims = self.cage_sizes[model]
        self.cage.dims = DimsConfig(x=dims.x, y=dims.y)
        self.cage.cage_type = model

    def stretch_cage_size(self, dx: float = 0.0, dy: float = 0.0) -> None:
        model = self.cage.cage_type
        dims = self.cage_sizes[model]
        self.cage.dims = DimsConfig(x=dims.x + dx, y=dims.y + dy)

    def generate(
        self,
        scenario_name: ScenarioName,
        selected_actors: Optional[List[ActorRole]] = None,
        time_scale: float = 1.0,
    ) -> ConfigureSimulation:
        actors = self._make_actor_configs(selected_actors)
        objective_msgs = [self.objectives[actor.objective].to_msg() for actor in actors]
        scenario = ScenarioConfig(
            name=scenario_name,
            cage=self.cage,
            background=self.background,
            actors=actors,
            fixtures=self.fixtures,
            physics_materials=[material for material in self.materials.values()],
            time_scale=time_scale,
        )
        if self.spectator_camera:
            scenario.main_cam = self.spectator_camera
        return ConfigureSimulation(scenario=scenario.to_msg(), objectives=objective_msgs)

    def register_actor(self, role: ActorRole, actor: ActorModel, objective_name: ObjectiveName) -> None:
        self.actors[role] = ActorConfig(
            name=role,
            model=actor,
            objective=objective_name,
        )

    def register_material(self, name: str, material: PhysicsMaterialsConfig) -> None:
        self.materials[name] = material

    def register_objectives(self, *objectives: ObjectiveConfig) -> None:
        for objective in objectives:
            self.objectives[objective.name] = objective

    def _make_actor_configs(self, selected_actors: Optional[List[ActorRole]] = None) -> List[ActorConfig]:
        configs: List[ActorConfig] = []
        if selected_actors is None:
            selected_actors = list(self.actors.keys())
        for actor_role in selected_actors:
            if actor_role not in self.actors:
                raise ValueError(f"Actor {actor_role} not registered.")
            configs.append(self.actors[actor_role])
        return configs

    def reset(self) -> None:
        self.actors.clear()
        self.objectives.clear()
        self.scenarios.clear()
        self.materials.clear()
        self.cage = CageConfig()
        self.background = BackgroundConfig()
        self.spectator_camera = None
        self.fixtures = FixturesConfig()
