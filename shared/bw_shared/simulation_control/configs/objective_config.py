# This file mirrors simulation/TrueBattleBotSim/Assets/Scripts/Scenarios/ObjectiveConfig.cs
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import List

import bw_interfaces.msg as bw_interfaces

from bw_shared.messages.dataclass_utils import from_dict, to_dict
from bw_shared.pid.config import PidConfig
from bw_shared.simulation_control.configs.ramsete_config import RamseteConfig
from bw_shared.simulation_control.configs.scenario_init_config import ScenarioInitConfig
from bw_shared.simulation_control.configs.sequence_element_config import SequenceElementConfig
from bw_shared.simulation_control.enums.follower_engine_type import FollowerEngineType
from bw_shared.simulation_control.enums.objective_name import ObjectiveName
from bw_shared.simulation_control.enums.objective_type import ObjectiveType


@dataclass
class ObjectiveConfig:
    name: ObjectiveName
    type: ObjectiveType = ObjectiveType.IDLE
    follower_engine: FollowerEngineType = FollowerEngineType.PID
    smooth_teleports: bool = False
    init: ScenarioInitConfig = field(default_factory=ScenarioInitConfig)
    sequence: List[SequenceElementConfig] = field(default_factory=list)

    overwrite_controller_config: bool = False
    linear_pid: PidConfig = field(default_factory=PidConfig)
    angular_pid: PidConfig = field(default_factory=PidConfig)
    ramsete: RamseteConfig = field(default_factory=RamseteConfig)

    def to_msg(self) -> bw_interfaces.ObjectiveConfig:
        return bw_interfaces.ObjectiveConfig(name=self.name, json_data=json.dumps(to_dict(self)))

    @classmethod
    def from_msg(cls, msg: bw_interfaces.ObjectiveConfig) -> ObjectiveConfig:
        data = json.loads(msg.json_data)
        data["name"] = msg.name
        return from_dict(cls, data)
