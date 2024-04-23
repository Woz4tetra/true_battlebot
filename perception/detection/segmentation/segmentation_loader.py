from typing import Union

from config.segmentation_config.instance_segmentation_config import InstanceSegmentationConfig
from config.segmentation_config.noop_segmentation_config import NoopSegmentationConfig
from config.segmentation_config.segmentation_types import SegmentationConfig

from .instance_segmentation import InstanceSegmentation
from .noop_segmentation import NoopSegmentation

InstanceImplementation = Union[InstanceSegmentation, NoopSegmentation]


def load_segmentation(config: SegmentationConfig) -> InstanceImplementation:
    return {InstanceSegmentationConfig: InstanceSegmentation, NoopSegmentationConfig: NoopSegmentation}[type(config)](
        config
    )
