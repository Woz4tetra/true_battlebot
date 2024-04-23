from typing import Union

from .instance_segmentation_config import InstanceSegmentationConfig
from .noop_segmentation_config import NoopSegmentationConfig

SegmentationConfig = Union[NoopSegmentationConfig, InstanceSegmentationConfig]
