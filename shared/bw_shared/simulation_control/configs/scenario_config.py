# This file mirrors simulation/TrueBattleBotSim/Assets/Scripts/Scenarios/ScenarioConfig.cs
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import List

import bw_interfaces.msg as bw_interfaces

from bw_shared.messages.dataclass_utils import from_dict, to_dict
from bw_shared.simulation_control.configs.actor_config import ActorConfig
from bw_shared.simulation_control.configs.background_config import BackgroundConfig
from bw_shared.simulation_control.configs.cage_config import CageConfig
from bw_shared.simulation_control.configs.camera_config import CameraConfig
from bw_shared.simulation_control.configs.fixtures_config import FixturesConfig
from bw_shared.simulation_control.configs.physics_material_config import PhysicsMaterialsConfig
from bw_shared.simulation_control.configs.scenario_init_config import ScenarioInitConfig
from bw_shared.simulation_control.enums.scenario_init_type import ScenarioInitType


@dataclass
class ScenarioConfig:
    name: str
    cage: CageConfig = field(default_factory=CageConfig)
    background: BackgroundConfig = field(default_factory=BackgroundConfig)

    main_cam: CameraConfig = field(
        default_factory=lambda: CameraConfig(
            init=ScenarioInitConfig(
                type=ScenarioInitType.WORLD, x=0.0, y=1.167, z=-2.05, roll=29.654, pitch=0.0, yaw=0.0
            )
        )
    )
    actors: List[ActorConfig] = field(default_factory=list)
    fixtures: FixturesConfig = field(default_factory=FixturesConfig)
    physics_materials: List[PhysicsMaterialsConfig] = field(default_factory=list)
    time_scale: float = 1.0

    def to_msg(self) -> bw_interfaces.ScenarioConfig:
        return bw_interfaces.ScenarioConfig(name=self.name, json_data=json.dumps(to_dict(self)))

    @classmethod
    def from_msg(cls, msg: bw_interfaces.ScenarioConfig) -> ScenarioConfig:
        data = json.loads(msg.json_data)
        data["name"] = msg.name
        return from_dict(cls, data)
