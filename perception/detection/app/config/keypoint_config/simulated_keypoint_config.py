from dataclasses import dataclass, field
from typing import Literal


@dataclass
class SimulatedKeypointConfig:
    type: Literal["SimulatedKeypoint"] = "SimulatedKeypoint"
    debug: bool = True
    model_to_system_labels: dict[str, str] = field(default_factory=dict)
