from dataclasses import dataclass, field
from typing import Literal


@dataclass
class SimulatedKeypointConfig:
    type: Literal["SimulatedKeypoint"] = "SimulatedKeypoint"
    debug: bool = True
    simulated_to_real_labels: dict[str, str] = field(default_factory=dict)
    radii: dict[str, float] = field(default_factory=dict)
    default_radius: float = 0.1
