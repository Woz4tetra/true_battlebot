from typing import Union

from app.config.camera.camera_types import CameraConfig
from app.config.camera.light_simulated_camera_config import LightSimulatedCameraConfig
from app.config.camera.noop_camera_config import NoopCameraConfig
from app.config.camera.simulated_camera_config import SimulatedCameraConfig
from app.config.camera.svo_playback_camera_config import SvoPlaybackCameraConfig
from app.config.camera.zed_camera_config import ZedCameraConfig
from app.config.config import Config
from app.container import Container
from bw_interfaces.msg import CageCorner, ControlRecording
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
from std_msgs.msg import Empty

from .light_simulated_camera import LightSimulatedCamera
from .noop_camera import NoopCamera
from .simulated_camera import SimulatedCamera
from .svo_playback_camera import SvoPlaybackCamera
from .zed_camera import ZedCamera

CameraImplementation = Union[
    ZedCamera,
    NoopCamera,
    SimulatedCamera,
    SvoPlaybackCamera,
    LightSimulatedCamera,
]


def make_simulated_camera(camera_config: SimulatedCameraConfig, container: Container) -> CameraImplementation:
    config = container.resolve(Config)
    ns = config.camera_topic.namespace

    color_image_sub = RosPollSubscriber(ns + "/rgb/image_raw", Image, buff_size=2 << 24)
    depth_image_sub = RosPollSubscriber(ns + "/depth/depth_registered", Image, buff_size=2 << 32)
    camera_info_sub = RosPollSubscriber(ns + "/rgb/camera_info", CameraInfo)
    depth_request_pub = RosPublisher(ns + "/depth/request", Empty)

    return SimulatedCamera(
        camera_config,
        config.camera_topic,
        color_image_sub,
        depth_image_sub,
        camera_info_sub,
        depth_request_pub,
    )


def make_zed_camera(camera_config: ZedCameraConfig, container: Container) -> CameraImplementation:
    config = container.resolve(Config)
    ns = config.camera_topic.namespace

    color_image_pub = RosPublisher(ns + "/rgb/image_raw", Image)
    camera_info_pub = RosPublisher(ns + "/rgb/camera_info", CameraInfo)
    imu_pub = RosPublisher(ns + "/imu/data", Imu)
    # compressed_image_pub = RosPublisher(ns + "/rgb/image_raw/compressed", CompressedImage)

    svo_record_sub = RosPollSubscriber(ns + "/record_svo", ControlRecording)

    return ZedCamera(
        config=camera_config,
        camera_topic_config=config.camera_topic,
        color_image_pub=color_image_pub,
        camera_info_pub=camera_info_pub,
        compressed_image_pub=None,
        imu_pub=imu_pub,
        record_svo_sub=svo_record_sub,
    )


def make_svo_camera(camera_config: SvoPlaybackCameraConfig, container: Container) -> CameraImplementation:
    config = container.resolve(Config)
    ns = config.camera_topic.namespace

    color_image_pub = RosPublisher(ns + "/rgb/image_raw", Image)
    camera_info_pub = RosPublisher(ns + "/rgb/camera_info", CameraInfo)
    imu_pub = RosPublisher(ns + "/imu/data", Imu)
    clock_pub = RosPublisher("/clock", Clock)

    svo_record_sub = RosPollSubscriber(ns + "/record_svo", ControlRecording)

    bag_topics = {
        "/set_cage_corner": CageCorner,
        "/manual_plane_request": Empty,
        "/camera_1/camera_info": CameraInfo,
        "/camera_1/image_rect": Image,
        "/camera_1/image_rect/compressed": CompressedImage,
    }
    bag_publisher = {topic: RosPublisher(topic, msg_type) for topic, msg_type in bag_topics.items()}

    return SvoPlaybackCamera(
        camera_config,
        config.camera_topic,
        color_image_pub,
        camera_info_pub,
        imu_pub,
        svo_record_sub,
        bag_publisher,
        clock_pub,
    )


def make_light_simulated_camera(
    camera_config: LightSimulatedCameraConfig, container: Container
) -> CameraImplementation:
    config = container.resolve(Config)
    ns = config.camera_topic.namespace

    camera_info_sub = RosPollSubscriber(ns + "/rgb/camera_info", CameraInfo)

    return LightSimulatedCamera(
        config=camera_config,
        camera_topic_config=config.camera_topic,
        camera_info_sub=camera_info_sub,
    )


def load_camera(config: CameraConfig, container: Container) -> CameraImplementation:
    if isinstance(config, SimulatedCameraConfig):
        return make_simulated_camera(config, container)
    elif isinstance(config, ZedCameraConfig):
        return make_zed_camera(config, container)
    elif isinstance(config, SvoPlaybackCameraConfig):
        return make_svo_camera(config, container)
    elif isinstance(config, NoopCameraConfig):
        return NoopCamera(config)
    elif isinstance(config, LightSimulatedCameraConfig):
        return make_light_simulated_camera(config, container)
    else:
        raise ValueError(f"Unsupported camera config type: {type(config)}")
