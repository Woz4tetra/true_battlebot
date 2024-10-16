from dataclasses import dataclass, field

from bw_behaviors.config.corner_mode_config import CornerModeConfig


@dataclass
class ContainerConfig:
    corner_offset: float = 0.25
    corner_mode: CornerModeConfig = field(default_factory=CornerModeConfig)
