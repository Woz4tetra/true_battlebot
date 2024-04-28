from typing import Union

from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.compressed_image import CompressedImage
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from roslibpy import Ros

from app.config.camera_config.camera_types import CameraConfig
from app.config.camera_config.noop_camera_config import NoopCameraConfig
from app.config.camera_config.simulated_camera_config import SimulatedCameraConfig
from app.config.camera_config.zed_camera_config import ZedCameraConfig
from app.config.config import Config
from app.container import Container

from .noop_camera import NoopCamera
from .simulated_camera import SimulatedCamera
from .zed_camera import ZedCamera

CameraImplementation = Union[ZedCamera, NoopCamera, SimulatedCamera]


def make_simulated_camera(camera_config: SimulatedCameraConfig, container: Container) -> CameraImplementation:
    ros = container.resolve(Ros)
    config = container.resolve(Config)
    ns = config.camera_topic.namespace

    color_image_sub = RosPollSubscriber(ros, ns + "/rgb/image_raw/compressed", CompressedImage)
    depth_image_sub = RosPollSubscriber(ros, ns + "/depth/depth_registered/compressed", CompressedImage)
    camera_info_sub = RosPollSubscriber(ros, ns + "/rgb/camera_info", CameraInfo)

    return SimulatedCamera(camera_config, config.camera_topic, color_image_sub, depth_image_sub, camera_info_sub)


def load_camera(config: CameraConfig, container: Container) -> CameraImplementation:
    if isinstance(config, SimulatedCameraConfig):
        return make_simulated_camera(config, container)
    elif isinstance(config, ZedCameraConfig):
        return ZedCamera(config)
    elif isinstance(config, NoopCameraConfig):
        return NoopCamera(config)
    else:
        raise ValueError(f"Unsupported camera config type: {type(config)}")
