from typing import Union

from .noop_segmentation_config import NoopSegmentationConfig
from .semantic_segmentation_config import SemanticSegmentationConfig
from .simulated_segmentation_config import SimulatedSegmentationConfig

SegmentationConfig = Union[
    NoopSegmentationConfig,
    SimulatedSegmentationConfig,
    SemanticSegmentationConfig,
]
