from typing import Union

from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image
from perception_tools.rosbridge.ros_factory import RosFactory

from app.config.camera_config.camera_types import CameraConfig
from app.config.camera_config.noop_camera_config import NoopCameraConfig
from app.config.camera_config.simulated_camera_config import SimulatedCameraConfig
from app.config.camera_config.zed_camera_config import ZedCameraConfig
from app.container import Container

from .noop_camera import NoopCamera
from .simulated_camera import SimulatedCamera
from .zed_camera import ZedCamera

CameraImplementation = Union[ZedCamera, NoopCamera, SimulatedCamera]


def make_simulated_camera(config: SimulatedCameraConfig, container: Container) -> CameraImplementation:
    ros_factory = container.resolve(RosFactory)

    ns = config.namespace
    color_image_sub = ros_factory.make_subscriber(ns + "/rgb/image_rect_color", Image)
    depth_image_sub = ros_factory.make_subscriber(ns + "/depth/depth_registered", Image)
    camera_info_sub = ros_factory.make_subscriber(ns + "/rgb/camera_info", CameraInfo)

    return SimulatedCamera(config, color_image_sub, depth_image_sub, camera_info_sub)


def load_camera(config: CameraConfig, container: Container) -> CameraImplementation:
    if isinstance(config, SimulatedCameraConfig):
        return make_simulated_camera(config, container)
    elif isinstance(config, ZedCameraConfig):
        return ZedCamera(config)
    elif isinstance(config, NoopCameraConfig):
        return NoopCamera(config)
    else:
        raise ValueError(f"Unsupported camera config type: {type(config)}")
