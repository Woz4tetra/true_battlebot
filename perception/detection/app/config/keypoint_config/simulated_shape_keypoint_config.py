from dataclasses import dataclass, field
from typing import Literal

from app.config.keypoint_config.pattern_finder_config.pattern_finder_types import PatternFinderConfig
from app.config.keypoint_config.pattern_finder_config.triangle_pattern_finder_config import TrianglePatternFinderConfig
from bw_shared.configs.model_to_system_labels_map import ModelToSystemLabelsMap


@dataclass
class SimulatedShapeKeypointConfig:
    type: Literal["SimulatedShapeKeypoint"] = "SimulatedShapeKeypoint"
    debug_image: bool = False
    model_to_system_labels: ModelToSystemLabelsMap = field(default_factory=ModelToSystemLabelsMap)
    pattern_finder: PatternFinderConfig = field(default_factory=TrianglePatternFinderConfig)
