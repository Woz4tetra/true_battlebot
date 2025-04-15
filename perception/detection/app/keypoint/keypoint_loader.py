from typing import Union

from app.config.config import Config
from app.config.keypoint_config.keypoint_to_object_converter_config import KeypointToObjectConverterConfig
from app.config.keypoint_config.keypoint_types import KeypointConfig
from app.config.keypoint_config.noop_keypoint_config import NoopKeypointConfig
from app.config.keypoint_config.simulated_keypoint_config import SimulatedKeypointConfig
from app.config.keypoint_config.yolo_keypoint_config import YoloKeypointConfig
from app.container import Container
from app.keypoint.ground_truth_manager import GroundTruthManager
from app.keypoint.keypoint_to_object_converter import KeypointToObjectConverter
from bw_interfaces.msg import EstimatedObjectArray
from bw_shared.configs.shared_config import SharedConfig
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from visualization_msgs.msg import MarkerArray

from .noop_keypoint import NoopKeypoint
from .simulated_keypoint import SimulatedKeypoint
from .yolo_keypoint import YoloKeypoint

KeypointImplementation = Union[NoopKeypoint, YoloKeypoint, SimulatedKeypoint]


def load_ground_truth_manager(container: Container) -> GroundTruthManager:
    main_config = container.resolve(Config)
    ns = main_config.camera_topic.namespace
    robots_sub = RosPollSubscriber(ns + "/ground_truth/robots", EstimatedObjectArray)
    return GroundTruthManager(robots_sub)


def load_keypoint(container: Container, config: KeypointConfig) -> KeypointImplementation:
    if isinstance(config, NoopKeypointConfig):
        return NoopKeypoint(config)
    elif isinstance(config, YoloKeypointConfig):
        return YoloKeypoint(config)
    elif isinstance(config, SimulatedKeypointConfig):
        if container.is_registered(GroundTruthManager):
            manager = container.resolve(GroundTruthManager)
        else:
            manager = load_ground_truth_manager(container)
            container.register(manager)
        return SimulatedKeypoint(config, manager)
    else:
        raise ValueError(f"Unknown keypoint config type: {type(config)}")


def load_keypoint_to_object_converter(
    container: Container, config: KeypointToObjectConverterConfig
) -> KeypointToObjectConverter:
    main_config = container.resolve(Config)
    shared_config = container.resolve(SharedConfig)
    namespace = main_config.camera_topic.namespace
    robot_pub = RosPublisher(namespace + "/estimation/robots", EstimatedObjectArray)
    robot_marker_pub = RosPublisher(namespace + "/estimation/robot_markers", MarkerArray)
    return KeypointToObjectConverter(config, shared_config.labels, robot_pub, robot_marker_pub)
