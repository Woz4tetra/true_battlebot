from dataclasses import dataclass, field
from typing import Literal

from app.config.keypoint_config.model_to_system_labels_config import ModelToSystemLabelsConfig


@dataclass
class SimulatedKeypointConfig:
    type: Literal["SimulatedKeypoint"] = "SimulatedKeypoint"
    debug: bool = True
    model_to_system_labels: ModelToSystemLabelsConfig = field(default_factory=ModelToSystemLabelsConfig)
