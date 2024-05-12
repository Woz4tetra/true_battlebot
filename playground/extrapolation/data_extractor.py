import os
import pickle

import rosbag
import rospy
import tf2_py
from apriltag_ros.msg import AprilTagDetectionArray
from bw_interfaces.msg import EstimatedObjectArray
from bw_shared.configs.shared_config import SharedConfig
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from object_data import ObjectData


def odom_callback(data: dict[str, list[PoseStamped]], topic: str, odometry: Odometry) -> None:
    key = topic.split("/")[1]
    if key not in data:
        data[key] = []
    data[key].append(PoseStamped(odometry.header, odometry.pose.pose))


def robot_callback(
    sensor_frames: set[str], measurements: dict[str, list[PoseStamped]], robots: EstimatedObjectArray
) -> None:
    for robot in robots.robots:
        key = robot.label
        if key not in measurements:
            measurements[key] = []
        measurements[key].append(PoseStamped(robot.header, robot.pose.pose))
        sensor_frames.add(robot.header.frame_id)


def tag_callback(
    sensor_frames: set[str],
    measurements: dict[str, list[PoseStamped]],
    robot_tag_ids: dict[int, str],
    tags: AprilTagDetectionArray,
) -> None:
    for detection in tags.detections:
        key = robot_tag_ids[detection.id[0]]
        if key not in measurements:
            measurements[key] = []
        measurements[key].append(PoseStamped(tags.header, detection.pose.pose.pose))
        sensor_frames.add(tags.header.frame_id)


def load_data(bag_path: str) -> ObjectData:
    pickle_path = os.path.splitext(bag_path)[0] + ".pkl"
    if os.path.isfile(pickle_path):
        with open(pickle_path, "rb") as f:
            return pickle.load(f)
    bag = rosbag.Bag(bag_path)

    robot_names = [
        "main_bot",
        "mini_bot",
        "opponent_1",
        "referee",
    ]
    ground_truth_topics = ["/" + name + "/ground_truth" for name in robot_names]
    filtered_topics = ["/" + name + "/odom" for name in robot_names]
    robot_measurement_topics = [
        "/camera_0/estimation/robots",
        "/camera_1/estimation/robots",
    ]
    tag_measurement_topics = [
        "/camera_1/tag_detections",
    ]
    ground_truth_data: dict[str, list[PoseStamped]] = {}
    filtered_data: dict[str, list[PoseStamped]] = {}
    measurements: dict[str, list[PoseStamped]] = {}
    config = SharedConfig.from_files()
    robot_tag_ids = {}
    for config in config.robots.robots:
        robot_tag_ids[config.up_id] = config.name
        robot_tag_ids[config.down_id] = config.name
    sensor_frames: set[str] = set()

    tf_buffer = tf2_py.BufferCore(cache_time=rospy.Duration(1000000))  # type: ignore
    for topic, msg, timestamp in bag.read_messages():
        if topic in ground_truth_topics:
            odom_callback(ground_truth_data, topic, msg)
        elif topic in filtered_topics:
            odom_callback(filtered_data, topic, msg)
        elif topic in robot_measurement_topics:
            robot_callback(sensor_frames, measurements, msg)
        elif topic in tag_measurement_topics:
            tag_callback(sensor_frames, measurements, robot_tag_ids, msg)
        elif topic == "/tf_static":
            for msg_tf in msg.transforms:
                tf_buffer.set_transform_static(msg_tf, "default_authority")
        elif topic == "/tf":
            for msg_tf in msg.transforms:
                tf_buffer.set_transform(msg_tf, "default_authority")

    sensor_transforms: dict[str, TransformStamped] = {}
    for child_frame_id in sensor_frames:
        transform = tf_buffer.lookup_transform_core("map", child_frame_id, rospy.Time(0))
        sensor_transforms[child_frame_id] = transform

    data = ObjectData(ground_truth_data, filtered_data, measurements, sensor_transforms)
    with open(pickle_path, "wb") as f:
        pickle.dump(data, f)
    return data
