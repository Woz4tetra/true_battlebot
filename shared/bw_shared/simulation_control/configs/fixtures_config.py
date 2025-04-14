from dataclasses import dataclass, field

from bw_shared.simulation_control.configs.even_lighting_config import EvenLightingConfig
from bw_shared.simulation_control.configs.spotlight_config import SpotlightConfig


@dataclass
class FixturesConfig:
    spotlight: SpotlightConfig = field(default_factory=SpotlightConfig)
    even_lighting: EvenLightingConfig = field(default_factory=EvenLightingConfig)
