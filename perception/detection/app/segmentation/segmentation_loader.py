from typing import Union

from app.config.segmentation.noop_segmentation_config import NoopSegmentationConfig
from app.config.segmentation.segmentation_types import SegmentationConfig
from app.config.segmentation.semantic_segmentation_config import SemanticSegmentationConfig
from app.config.segmentation.simulated_segmentation_config import SimulatedSegmentationConfig
from app.container import Container
from app.segmentation.load_simulated_segmentation_manager import load_simulated_segmentation_manager
from app.segmentation.semantic_segmentation import SemanticSegmentation

from .noop_segmentation import NoopSegmentation
from .simulated_segmentation import SimulatedSegmentation

SegmentationImplementation = Union[
    NoopSegmentation,
    SimulatedSegmentation,
    SemanticSegmentation,
]


def load_simulated_segmentation(container: Container, config: SimulatedSegmentationConfig) -> SimulatedSegmentation:
    manager = load_simulated_segmentation_manager(container)
    return SimulatedSegmentation(config, manager)


def load_segmentation(container: Container, config: SegmentationConfig) -> SegmentationImplementation:
    if isinstance(config, NoopSegmentationConfig):
        return NoopSegmentation(config)
    elif isinstance(config, SimulatedSegmentationConfig):
        return load_simulated_segmentation(container, config)
    elif isinstance(config, SemanticSegmentationConfig):
        return SemanticSegmentation(config)
    else:
        raise ValueError(f"Unknown segmentation config type: {type(config)}")
