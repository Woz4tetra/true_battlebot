from dataclasses import dataclass, field
from typing import Literal

from bw_shared.configs.model_to_system_labels_map import ModelToSystemLabelsMap


@dataclass
class SimulatedKeypointConfig:
    type: Literal["SimulatedKeypoint"] = "SimulatedKeypoint"
    debug: bool = True
    model_to_system_labels: ModelToSystemLabelsMap = field(default_factory=ModelToSystemLabelsMap)
