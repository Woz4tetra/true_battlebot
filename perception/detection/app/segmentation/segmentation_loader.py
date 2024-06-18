from typing import Union

from bw_interfaces.msg import SegmentationInstanceArray
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from sensor_msgs.msg import Image

from app.config.config import Config
from app.config.segmentation_config.instance_segmentation_config import InstanceSegmentationConfig
from app.config.segmentation_config.noop_segmentation_config import NoopSegmentationConfig
from app.config.segmentation_config.segmentation_types import SegmentationConfig
from app.config.segmentation_config.semantic_segmentation_config import SemanticSegmentationConfig
from app.config.segmentation_config.simulated_segmentation_config import SimulatedSegmentationConfig
from app.container import Container
from app.segmentation.semantic_segmentation import SemanticSegmentation
from app.segmentation.simulated_segmentation_manager import SimulatedSegmentationManager

from .instance_segmentation import InstanceSegmentation
from .noop_segmentation import NoopSegmentation
from .simulated_segmentation import SimulatedSegmentation

SegmentationImplementation = Union[
    InstanceSegmentation,
    NoopSegmentation,
    SimulatedSegmentation,
    SemanticSegmentation,
]


def load_simulated_segmentation_manager(container: Container) -> SimulatedSegmentationManager:
    config = container.resolve(Config)
    namespace = config.camera_topic.namespace
    sim_segmentation_image_sub = RosPollSubscriber(namespace + "/layer/image_raw", Image)
    simulated_segmentation_sub = RosPollSubscriber(namespace + "/simulated_segmentation", SegmentationInstanceArray)
    return SimulatedSegmentationManager(sim_segmentation_image_sub, simulated_segmentation_sub)


def load_segmentation(container: Container, config: SegmentationConfig) -> SegmentationImplementation:
    if isinstance(config, InstanceSegmentationConfig):
        return InstanceSegmentation(config)
    elif isinstance(config, NoopSegmentationConfig):
        return NoopSegmentation(config)
    elif isinstance(config, SimulatedSegmentationConfig):
        if container.is_registered(SimulatedSegmentationManager):
            manager = container.resolve(SimulatedSegmentationManager)
        else:
            manager = load_simulated_segmentation_manager(container)
            container.register(manager)
        return SimulatedSegmentation(config, manager)
    elif isinstance(config, SemanticSegmentationConfig):
        return SemanticSegmentation(config)
    else:
        raise ValueError(f"Unknown segmentation config type: {type(config)}")
