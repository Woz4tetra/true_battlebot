from typing import Union

from app.config.config import Config
from app.config.keypoint_config.keypoint_to_object_converter_config import KeypointToObjectConverterConfig
from app.config.keypoint_config.keypoint_types import KeypointConfig
from app.config.keypoint_config.noop_keypoint_config import NoopKeypointConfig
from app.config.keypoint_config.pattern_finder_config.noop_pattern_finder_config import NoopPatternFinderConfig
from app.config.keypoint_config.pattern_finder_config.pattern_finder_types import PatternFinderConfig
from app.config.keypoint_config.pattern_finder_config.triangle_pattern_finder_config import TrianglePatternFinderConfig
from app.config.keypoint_config.simulated_keypoint_config import SimulatedKeypointConfig
from app.config.keypoint_config.simulated_shape_keypoint_config import SimulatedShapeKeypointConfig
from app.config.keypoint_config.yolo_keypoint_config import YoloKeypointConfig
from app.container import Container
from app.keypoint.ground_truth_manager import GroundTruthManager
from app.keypoint.keypoint_to_object_converter import KeypointToObjectConverter
from app.segmentation.load_simulated_segmentation_manager import load_simulated_segmentation_manager
from bw_interfaces.msg import EstimatedObjectArray
from bw_shared.configs.shared_config import SharedConfig
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from visualization_msgs.msg import MarkerArray

from .noop_keypoint import NoopKeypoint
from .pattern_finder.noop_pattern_finder import NoopPatternFinder
from .pattern_finder.triangle_pattern_finder import TrianglePatternFinder
from .simulated_keypoint import SimulatedKeypoint
from .simulated_shape_keypoint import SimulatedShapeKeypoint
from .yolo_keypoint import YoloKeypoint

KeypointImplementation = Union[NoopKeypoint, YoloKeypoint, SimulatedKeypoint, SimulatedShapeKeypoint]
PatternFinderImplementation = Union[NoopPatternFinder, TrianglePatternFinder]


def load_ground_truth_manager(container: Container) -> GroundTruthManager:
    main_config = container.resolve(Config)
    ns = main_config.camera_topic.namespace
    robots_sub = RosPollSubscriber(ns + "/ground_truth/robots", EstimatedObjectArray)
    return GroundTruthManager(robots_sub)


def load_pattern_finder(container: Container, config: PatternFinderConfig) -> PatternFinderImplementation:
    if isinstance(config, NoopPatternFinderConfig):
        return NoopPatternFinder(config)
    elif isinstance(config, TrianglePatternFinderConfig):
        return TrianglePatternFinder(config)
    else:
        raise ValueError(f"Unknown pattern finder config type: {type(config)}")


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
    elif isinstance(config, SimulatedShapeKeypointConfig):
        manager = load_simulated_segmentation_manager(container)
        pattern_finder = load_pattern_finder(container, config.pattern_finder)
        return SimulatedShapeKeypoint(config, manager, pattern_finder)
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
