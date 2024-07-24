from dataclasses import dataclass, field
from typing import Literal

from bw_shared.enums.label import Label, ModelLabel


@dataclass
class SimulatedKeypointConfig:
    type: Literal["SimulatedKeypoint"] = "SimulatedKeypoint"
    debug: bool = True
    model_to_system_labels: dict[ModelLabel, Label] = field(default_factory=dict)
