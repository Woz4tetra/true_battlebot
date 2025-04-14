from dataclasses import dataclass, field

from bw_shared.enums.cage_model import CageModel
from bw_shared.simulation_control.configs.dims_config import DimsConfig


@dataclass
class CageConfig:
    dims: DimsConfig = field(default_factory=DimsConfig)
    display_readout: bool = True
    cage_type: CageModel = CageModel.NHRL_3LB_CAGE
