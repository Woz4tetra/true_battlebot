from dataclasses import dataclass
from typing import Literal


@dataclass
class SimulatedFieldFilterConfig:
    type: Literal["SimulatedFieldFilter"] = "SimulatedFieldFilter"
