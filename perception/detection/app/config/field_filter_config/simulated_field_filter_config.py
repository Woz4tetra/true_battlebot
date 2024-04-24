from dataclasses import dataclass


@dataclass
class SimulatedFieldFilterConfig:
    type: str = "SimulatedFieldFilter"
    namespace: str = "camera_0"
