import time
from typing import cast

import rospy
import toml
from app.config.field_label_tool_config.field_label_config import FieldLabelConfig
from app.config.field_label_tool_config.nhrl_cam_label_config import NhrlCamLabelConfig
from app.field_label.command_line_args import (
    BagCommandLineArgs,
    CommandLineArgs,
    NhrlCamCommandLineArgs,
    SvoCommandLineArgs,
    TopicCommandLineArgs,
)
from app.field_label.field_label_app import FieldLabelApp
from app.field_label.nhrl_cam_label_app import NhrlCamLabelApp
from app.field_label.util.load_from_bag import load_from_bag
from app.field_label.util.load_from_svo import load_from_svo
from app.field_label.util.load_from_topics import load_from_topics
from app.field_label.util.load_from_video import load_from_video
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.transform3d import Transform3D
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import PointCloud2 as RosPointCloud
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener


def load_field_label_config(path: str) -> FieldLabelConfig:
    with open(path, "r") as file:
        data = toml.load(file)
        return FieldLabelConfig.from_dict(data)


def run_bag(args: BagCommandLineArgs) -> None:
    config = load_field_label_config(args.config)
    camera_data, tf_pointcloud_from_camera = load_from_bag(
        args.bag_file, config.cloud_topic, config.image_topic, config.info_topic
    )
    app = FieldLabelApp(config, args, camera_data, tf_pointcloud_from_camera)
    app.label_camera_data()


def run_topic(args: TopicCommandLineArgs) -> None:
    rospy.init_node("field_label_app", disable_signals=True)
    config = load_field_label_config(args.config)
    cloud_subscriber = RosPollSubscriber(config.cloud_topic, RosPointCloud)
    image_subscriber = RosPollSubscriber(config.image_topic, RosImage)
    info_subscriber = RosPollSubscriber(config.info_topic, CameraInfo)
    request_publisher = RosPublisher(config.field_request_topic, Empty)
    response_publisher = RosPublisher(config.field_response_topic, EstimatedObject)
    tf_buffer = Buffer()
    TransformListener(tf_buffer)

    time.sleep(2.0)  # Wait for subscribers to connect

    print("Requesting camera data")
    request_publisher.publish(Empty())

    camera_data, tf_pointcloud_from_camera = load_from_topics(
        cloud_subscriber, image_subscriber, info_subscriber, tf_buffer
    )
    app = FieldLabelApp(config, args, camera_data, tf_pointcloud_from_camera)
    response = app.label_camera_data()
    if response is not None:
        response_publisher.publish(response)


def load_nhrl_label_config(path: str) -> NhrlCamLabelConfig:
    with open(path, "r") as file:
        data = toml.load(file)
        return NhrlCamLabelConfig.from_dict(data)


def run_video(args: NhrlCamCommandLineArgs) -> None:
    config = load_nhrl_label_config(args.config)
    video_frame = load_from_video(args.video_file)
    tf_pointcloud_from_camera = Transform3D.identity()
    camera_data = load_from_svo(args.svo_file, config.svo_start_time)
    app = NhrlCamLabelApp(config, args, camera_data, tf_pointcloud_from_camera, video_frame)
    app.label_camera_data()


def run_svo(args: SvoCommandLineArgs) -> None:
    config = load_field_label_config(args.config)
    camera_data = load_from_svo(args.svo_file, config.start_time)
    tf_pointcloud_from_camera = Transform3D.identity()
    app = FieldLabelApp(config, args, camera_data, tf_pointcloud_from_camera)
    app.label_camera_data()


def run_app(args: CommandLineArgs) -> None:
    match args.command:
        case "bag":
            run_bag(cast(BagCommandLineArgs, args))
        case "topic":
            run_topic(cast(TopicCommandLineArgs, args))
        case "svo":
            run_svo(cast(SvoCommandLineArgs, args))
        case "nhrl":
            run_video(cast(NhrlCamCommandLineArgs, args))
        case _:
            raise RuntimeError(f"Unknown command: {args.command}")
