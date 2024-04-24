from typing import Union

from perception_tools.messages.camera.image import Image
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray
from perception_tools.rosbridge.ros_factory import RosFactory

from app.config.segmentation_config.instance_segmentation_config import InstanceSegmentationConfig
from app.config.segmentation_config.noop_segmentation_config import NoopSegmentationConfig
from app.config.segmentation_config.segmentation_types import SegmentationConfig
from app.config.segmentation_config.simulated_segmentation_config import SimulatedSegmentationConfig
from app.container import Container

from .instance_segmentation import InstanceSegmentation
from .noop_segmentation import NoopSegmentation
from .simulated_segmentation import SimulatedSegmentation

SegmentationImplementation = Union[InstanceSegmentation, NoopSegmentation, SimulatedSegmentation]


def load_segmentation(container: Container, config: SegmentationConfig) -> SegmentationImplementation:
    if isinstance(config, InstanceSegmentationConfig):
        return InstanceSegmentation(config)
    elif isinstance(config, NoopSegmentationConfig):
        return NoopSegmentation(config)
    elif isinstance(config, SimulatedSegmentationConfig):
        ros_factory = container.resolve(RosFactory)
        namespace = config.namespace
        sim_segmentation_image_sub = ros_factory.make_subscriber(namespace + "/segmentation_image", Image)
        segmentation_pub = ros_factory.make_publisher(namespace + "/segmentation", SegmentationInstanceArray)
        simulated_segmentation_sub = ros_factory.make_subscriber(
            namespace + "/simulated_segmentation", SegmentationInstanceArray
        )
        return SimulatedSegmentation(config, sim_segmentation_image_sub, segmentation_pub, simulated_segmentation_sub)
    else:
        raise ValueError(f"Unknown segmentation config type: {type(config)}")
