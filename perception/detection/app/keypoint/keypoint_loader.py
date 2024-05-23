from typing import Union

from app.config.config import Config
from app.config.keypoint_config.keypoint_types import KeypointConfig
from app.config.keypoint_config.noop_keypoint_config import NoopKeypointConfig
from app.config.keypoint_config.simulated_keypoint_config import SimulatedKeypointConfig
from app.config.keypoint_config.yolo_keypoint_config import YoloKeypointConfig
from app.container import Container
from app.keypoint.ground_truth_manager import GroundTruthManager
from bw_interfaces.msg import EstimatedObjectArray
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from sensor_msgs.msg import CameraInfo

from .noop_keypoint import NoopKeypoint
from .simulated_keypoint import SimulatedKeypoint
from .yolo_keypoint import YoloKeypoint

KeypointImplementation = Union[NoopKeypoint, YoloKeypoint, SimulatedKeypoint]


def load_ground_truth_manager(container: Container, config: SimulatedKeypointConfig) -> GroundTruthManager:
    main_config = container.resolve(Config)
    ns = main_config.camera_topic.namespace
    camera_info_sub = RosPollSubscriber(ns + "/rgb/camera_info", CameraInfo)
    robots_sub = RosPollSubscriber("/camera_0/ground_truth/robots", EstimatedObjectArray)
    return GroundTruthManager(robots_sub, camera_info_sub)


def load_keypoint(container: Container, config: KeypointConfig) -> KeypointImplementation:
    if isinstance(config, NoopKeypointConfig):
        return NoopKeypoint(config)
    elif isinstance(config, YoloKeypointConfig):
        return YoloKeypoint(config)
    elif isinstance(config, SimulatedKeypointConfig):
        if container.is_registered(GroundTruthManager):
            manager = container.resolve(GroundTruthManager)
        else:
            manager = load_ground_truth_manager(container, config)
            container.register(manager)
        return SimulatedKeypoint(config, manager)
    else:
        raise ValueError(f"Unknown keypoint config type: {type(config)}")
