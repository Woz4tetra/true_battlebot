from typing import Union

from app.config.config import Config
from app.config.segmentation.noop_segmentation_config import NoopSegmentationConfig
from app.config.segmentation.segmentation_types import SegmentationConfig
from app.config.segmentation.semantic_segmentation_config import SemanticSegmentationConfig
from app.config.segmentation.simulated_segmentation_config import SimulatedSegmentationConfig
from app.container import Container
from app.segmentation.semantic_segmentation import SemanticSegmentation
from app.segmentation.simulated_segmentation_manager import SimulatedSegmentationManager
from bw_interfaces.msg import SegmentationInstanceArray
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

from .noop_segmentation import NoopSegmentation
from .simulated_segmentation import SimulatedSegmentation

SegmentationImplementation = Union[
    NoopSegmentation,
    SimulatedSegmentation,
    SemanticSegmentation,
]


def load_simulated_segmentation_manager(container: Container) -> SimulatedSegmentationManager:
    config = container.resolve(Config)
    namespace = config.camera_topic.namespace
    sim_segmentation_image_sub = RosPollSubscriber(namespace + "/layer/image_raw", Image, buff_size=2 << 24)
    simulated_segmentation_sub = RosPollSubscriber(namespace + "/simulated_segmentation", SegmentationInstanceArray)
    layer_request_pub = RosPublisher(namespace + "/layer/request", Empty)

    return SimulatedSegmentationManager(sim_segmentation_image_sub, simulated_segmentation_sub, layer_request_pub)


def load_simulated_segmentation(container: Container, config: SimulatedSegmentationConfig) -> SimulatedSegmentation:
    if container.is_registered(SimulatedSegmentationManager):
        manager = container.resolve(SimulatedSegmentationManager)
    else:
        manager = load_simulated_segmentation_manager(container)
        container.register(manager)
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
