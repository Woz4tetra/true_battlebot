import time
from typing import cast

import rospy
import toml
from app.field_label.command_line_args import (
    BagCommandLineArgs,
    CommandLineArgs,
    TopicCommandLineArgs,
    VideoCommandLineArgs,
)
from app.field_label.field_label_app import FieldLabelApp
from app.field_label.field_label_config import FieldLabelConfig
from app.field_label.load_from_bag import load_from_bag
from app.field_label.load_from_topics import load_from_topics
from app.field_label.load_from_video import load_from_video
from bw_interfaces.msg import EstimatedObject
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
    app = FieldLabelApp(config, args)
    camera_data, tf_pointcloud_from_camera = load_from_bag(
        args.bag_file, config.cloud_topic, config.image_topic, config.info_topic
    )
    app.label_camera_data(camera_data, tf_pointcloud_from_camera)


def run_topic(args: TopicCommandLineArgs) -> None:
    rospy.init_node("field_label_app", disable_signals=True)
    config = load_field_label_config(args.config)
    app = FieldLabelApp(config, args)
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
    response = app.label_camera_data(camera_data, tf_pointcloud_from_camera)
    if response is not None:
        response_publisher.publish(response)


def run_video(args: VideoCommandLineArgs) -> None:
    config = load_field_label_config(args.config)
    app = FieldLabelApp(config, args)
    video_frame = load_from_video(args.video_file)
    camera_data, tf_pointcloud_from_camera = load_from_bag(
        args.bag_file, config.cloud_topic, config.image_topic, config.info_topic
    )
    app.label_camera_data(camera_data, tf_pointcloud_from_camera)


def run_app(args: CommandLineArgs) -> None:
    match args.command:
        case "bag":
            run_bag(cast(BagCommandLineArgs, args))
        case "topic":
            run_topic(cast(TopicCommandLineArgs, args))
        case "video":
            run_video(cast(VideoCommandLineArgs, args))
        case _:
            raise RuntimeError(f"Unknown command: {args.command}")