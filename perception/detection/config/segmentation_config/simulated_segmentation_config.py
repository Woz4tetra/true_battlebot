from dataclasses import dataclass


@dataclass
class SimulatedSegmentationConfig:
    type: str = "SimulatedSegmentation"
    separate_friendlies: bool = True
    debug: bool = False
