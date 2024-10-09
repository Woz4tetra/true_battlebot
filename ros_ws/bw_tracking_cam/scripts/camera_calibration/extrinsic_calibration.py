#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass, field
from glob import glob
from typing import Optional, Union

import argcomplete
import cv2
import numpy as np
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from bw_shared.camera_calibration.board_config import BoardConfig
from bw_shared.geometry.camera.image_rectifier import ImageRectifier
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.transform3d import Transform3D
from bw_tools.tag_detection.bundle_detector import compute_pose_ransac
from bw_tools.tag_detection.draw_helpers import project_point_array_to_pixel
from cv2 import aruco
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rosbag.bag import Bag
from sensor_msgs.msg import CameraInfo, Image

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
CV_BRIDGE = CvBridge()


@dataclass(frozen=True, eq=True)
class CameraTopic:
    image_topic: str
    camera_info_topic: str
    ground_truth_topic: Optional[str] = None
    is_rectified: bool = False


@dataclass
class CameraData:
    topic: CameraTopic
    image: np.ndarray
    camera_info: CameraInfo
    ground_truth: Optional[AprilTagDetectionArray] = None

    def get_rectified(self) -> CameraData:
        if self.topic.is_rectified:
            return self
        rectifier = ImageRectifier(self.camera_info)
        rectified_info = rectifier.get_rectified_info()
        rectified_image = rectifier.rectify(self.image)
        rectified_topic = CameraTopic(
            self.topic.image_topic, self.topic.camera_info_topic, self.topic.ground_truth_topic, is_rectified=True
        )

        return CameraData(rectified_topic, rectified_image, rectified_info, self.ground_truth)


CameraCalibrationData = dict[CameraTopic, CameraData]


@dataclass
class ExstrinsicCalibrationConfig:
    detector_params: aruco.DetectorParameters = field(default_factory=aruco.DetectorParameters)
    charuco_params: aruco.CharucoParameters = field(default_factory=aruco.CharucoParameters)
    refine_params: aruco.RefineParameters = field(default_factory=aruco.RefineParameters)

    board: BoardConfig = field(default_factory=BoardConfig)


@dataclass
class AppData:
    camera_0: CameraTopic
    camera_1: CameraTopic
    detector: aruco.CharucoDetector
    config: ExstrinsicCalibrationConfig


def load_images(topics: list[CameraTopic], bag: Bag) -> CameraCalibrationData:
    all_data: dict[Optional[str], Optional[Union[Image, CameraInfo, AprilTagDetectionArray]]] = {}
    all_data.update({topic.image_topic: None for topic in topics})
    all_data.update({topic.camera_info_topic: None for topic in topics})
    all_data.update({topic.ground_truth_topic: None for topic in topics if topic.ground_truth_topic is not None})
    all_topics = [topic for topic in all_data.keys()]

    for topic, msg, _ in bag.read_messages(topics=all_topics):  # type: ignore
        if topic in all_data and all_data[topic] is None:
            all_data[topic] = msg

    data: CameraCalibrationData = {}
    for topic in topics:
        image_msg = all_data[topic.image_topic]
        camera_info = all_data[topic.camera_info_topic]
        ground_truth = all_data.get(topic.ground_truth_topic, None)
        if not isinstance(image_msg, Image) or not isinstance(camera_info, CameraInfo):
            raise ValueError(f"Missing data for topic {topic}")
        if ground_truth is not None and not isinstance(ground_truth, AprilTagDetectionArray):
            raise ValueError(f"Invalid ground truth data for topic {topic}")
        image = CV_BRIDGE.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

        data[topic] = CameraData(topic, image, camera_info, ground_truth)

    return data


def make_ransac_params() -> cv2.UsacParams:
    microsac_params = cv2.UsacParams()
    microsac_params.threshold = 5.0
    microsac_params.confidence = 0.99999
    microsac_params.score = cv2.SCORE_METHOD_MSAC
    microsac_params.maxIterations = 10_000
    microsac_params.loIterations = 100
    microsac_params.loMethod = cv2.LOCAL_OPTIM_GC

    microsac_params.final_polisher = cv2.LSQ_POLISHER
    microsac_params.final_polisher_iterations = 10_000

    return microsac_params


def compute_board_pose(
    camera_data: CameraData, detector: aruco.CharucoDetector, config: ExstrinsicCalibrationConfig
) -> Optional[Transform3D]:
    rectified = camera_data.get_rectified()
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(rectified.image)

    if charuco_corners is None or charuco_ids is None:
        raise RuntimeError("No charuco corners found")
    object_points, image_points = config.board.board.matchImagePoints(charuco_corners, charuco_ids)  # type: ignore

    return compute_pose_ransac(rectified.camera_info, make_ransac_params(), image_points, object_points)


def draw_points_in_image(
    image: np.ndarray, camera_info: CameraInfo, points: np.ndarray, color: tuple[int, int, int]
) -> None:
    tag_pixels = project_point_array_to_pixel(points, camera_info)
    tag_pixels = tag_pixels.astype(int)
    for tag_pixel in tag_pixels:
        cv2.circle(image, tag_pixel, 3, color, -1)


def compute_extrinsic_calibration(app: AppData, camera_data: CameraCalibrationData) -> None:
    window_name = "Extrinsic Calibration"
    cv2.namedWindow(window_name)

    camera_0 = app.camera_0
    camera_1 = app.camera_1
    detector = app.detector
    config = app.config
    grid_center_in_tag = Transform3D.from_position_and_rpy(
        Vector3(config.board.all_tag_width / 2, config.board.all_tag_width / 2, 0)
    )
    tf_board_from_camera0 = compute_board_pose(camera_data[camera_0], detector, config)
    if tf_board_from_camera0 is None:
        raise RuntimeError("Failed to compute pose for origin camera")
    tf_boardcenter_from_camera0 = grid_center_in_tag.forward_by(tf_board_from_camera0)

    tf_board_from_camera1 = compute_board_pose(camera_data[camera_1], detector, config)
    if tf_board_from_camera1 is None:
        raise RuntimeError("Failed to compute pose for relative camera")
    tf_boardcenter_from_camera1 = grid_center_in_tag.forward_by(tf_board_from_camera1)

    tf_camera0_from_camera1 = tf_board_from_camera0.inverse().forward_by(tf_board_from_camera1)

    tf_camera1_from_camera0 = tf_camera0_from_camera1.inverse()
    transform_properties = (
        tf_camera1_from_camera0.position.x,
        tf_camera1_from_camera0.position.y,
        tf_camera1_from_camera0.position.z,
        tf_camera1_from_camera0.quaternion.x,
        tf_camera1_from_camera0.quaternion.y,
        tf_camera1_from_camera0.quaternion.z,
        tf_camera1_from_camera0.quaternion.w,
    )
    properties_str = " ".join([f"{p:.6f}" for p in transform_properties])

    print(f"camera_0 pose: {tf_boardcenter_from_camera0}")
    print(f"camera_1 pose: {tf_boardcenter_from_camera1}")
    print(f"camera_1 pose from camera_0 camera: {tf_camera1_from_camera0}")
    print(f"static transform: {properties_str}")

    show_image = np.copy(camera_data[camera_1].image)
    show_info = camera_data[camera_1].camera_info
    grid_in_tag = config.board.grid_points

    ground_truth_camera0 = camera_data[camera_0].ground_truth
    ground_truth_camera1 = camera_data[camera_1].ground_truth
    if ground_truth_camera0 is not None and ground_truth_camera1 is not None:
        if len(ground_truth_camera0.detections) > 1:
            print("WARNING: Ground truth poses are not supported for multiple detections")
        if len(ground_truth_camera0.detections) == 0:
            raise RuntimeError("No ground truth poses found for camera_0")
        ground_truth_camera0_detection = ground_truth_camera0.detections[0]
        if len(ground_truth_camera1.detections) > 1:
            print("WARNING: Ground truth poses are not supported for multiple detections")
        if len(ground_truth_camera1.detections) == 0:
            raise RuntimeError("No ground truth poses found for camera_1")
        ground_truth_camera1_detection = ground_truth_camera1.detections[0]
        ground_tf_camera0_from_board = Transform3D.from_pose_msg(ground_truth_camera0_detection.pose.pose.pose)
        ground_tf_camera1_from_board = Transform3D.from_pose_msg(ground_truth_camera1_detection.pose.pose.pose)
        ground_tf_camera0_from_camera1 = ground_tf_camera0_from_board.inverse().forward_by(ground_tf_camera1_from_board)
        ground_tf_camera1_from_camera0 = ground_tf_camera0_from_camera1.inverse()

        print(f"Ground truth camera_0 pose: {ground_tf_camera0_from_board}")
        print(f"Ground truth camera_1 pose: {ground_tf_camera1_from_board}")
        print(f"Ground truth camera_1 pose from camera_0 camera: {ground_tf_camera1_from_camera0}")

        grid_points_in_camera1_ground = points_transform_by(
            config.board.get_grid_points(anchor=(-1, -1)), ground_tf_camera1_from_board.tfmat
        )
        draw_points_in_image(show_image, show_info, grid_points_in_camera1_ground, (255, 0, 0))

    grid_points_in_camera0 = points_transform_by(grid_in_tag, tf_board_from_camera0.tfmat)
    grid_points_in_camera1 = points_transform_by(grid_points_in_camera0, tf_camera0_from_camera1.tfmat)
    grid_points_in_camera1_original = points_transform_by(grid_in_tag, tf_board_from_camera1.tfmat)

    draw_points_in_image(show_image, show_info, grid_points_in_camera1, (0, 255, 0))
    draw_points_in_image(show_image, show_info, grid_points_in_camera1_original, (0, 0, 255))

    cv2.imshow(window_name, show_image)
    key = chr(cv2.waitKey(1) & 0xFF)
    if key == "q":
        cv2.destroyAllWindows()
        sys.exit(0)


def compute_calibration_from_bag(bag_path: str, app: AppData):
    bag = Bag(bag_path)
    camera_data = load_images([app.camera_0, app.camera_1], bag)
    bag.close()
    compute_extrinsic_calibration(app, camera_data)


def image_callback(camera_data: CameraCalibrationData, topic: CameraTopic, msg: Image) -> None:
    if topic not in camera_data:
        camera_data[topic] = CameraData(topic, np.array([]), CameraInfo())
    camera_data[topic].image = CV_BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def info_callback(camera_data: CameraCalibrationData, topic: CameraTopic, msg: CameraInfo) -> None:
    if topic not in camera_data:
        camera_data[topic] = CameraData(topic, np.array([]), CameraInfo())
    camera_data[topic].camera_info = msg


def ground_truth_callback(camera_data: CameraCalibrationData, topic: CameraTopic, msg: AprilTagDetectionArray) -> None:
    if topic not in camera_data:
        camera_data[topic] = CameraData(topic, np.array([]), CameraInfo())
    camera_data[topic].ground_truth = msg


def compute_calibration_live(app: AppData):
    rospy.init_node("extrinsic_calibration")
    camera_data: CameraCalibrationData = {}
    rospy.Subscriber(
        app.camera_0.image_topic,
        Image,
        callback=lambda msg, data=camera_data, topic=app.camera_0: image_callback(data, topic, msg),
    )
    rospy.Subscriber(
        app.camera_0.camera_info_topic,
        CameraInfo,
        callback=lambda msg, data=camera_data, topic=app.camera_0: info_callback(data, topic, msg),
    )
    rospy.Subscriber(
        app.camera_1.image_topic,
        Image,
        callback=lambda msg, data=camera_data, topic=app.camera_1: image_callback(data, topic, msg),
    )
    rospy.Subscriber(
        app.camera_1.camera_info_topic,
        CameraInfo,
        callback=lambda msg, data=camera_data, topic=app.camera_1: info_callback(data, topic, msg),
    )
    rospy.Subscriber(
        app.camera_0.ground_truth_topic,
        AprilTagDetectionArray,
        callback=lambda msg, data=camera_data, topic=app.camera_0: ground_truth_callback(data, topic, msg),
    )
    rospy.Subscriber(
        app.camera_1.ground_truth_topic,
        AprilTagDetectionArray,
        callback=lambda msg, data=camera_data, topic=app.camera_1: ground_truth_callback(data, topic, msg),
    )

    print("Waiting for calibration data")
    rospy.sleep(2.0)
    while not rospy.is_shutdown():
        try:
            compute_extrinsic_calibration(app, camera_data)
        except (RuntimeError, KeyError) as e:
            print(f"Failed to compute calibration. {e.__class__.__name__}: {e}")
            rospy.sleep(1.0)
            continue
        rospy.sleep(1.0)


def list_files(root_dir: str, extension: str) -> dict[str, str]:
    search_dir = os.path.abspath(root_dir)
    paths = [f for f in glob(os.path.join(search_dir, "**", f"*.{extension}"), recursive=True)]
    files = {f.replace(search_dir + "/", ""): f for f in paths}
    files[""] = ""
    return files


def main() -> None:
    parser = argparse.ArgumentParser()

    board_options = list_files(SCRIPT_DIR, "toml")
    bag_options = list_files("/data/bags", "bag")

    parser.add_argument("board_config", type=str, choices=board_options.keys(), help="path to board config")
    parser.add_argument(
        "-p",
        "--parameters",
        type=str,
        default=os.path.join(SCRIPT_DIR, "calibration_parameters.json"),
        help="calibration parameter file",
    )

    subparsers = parser.add_subparsers(dest="command")
    subparsers.add_parser("topic")

    bag_parser = subparsers.add_parser("bag")
    bag_parser.add_argument("bag_path", type=str, choices=bag_options.keys(), help="path to bag")

    argcomplete.autocomplete(parser)

    args = parser.parse_args()

    if args.command == "topic":
        is_live = True
        bag_path = ""
    else:
        is_live = False
        bag_path = bag_options[args.bag_path]
    parameter_path = args.parameters
    board_config_path = board_options[args.board_config]

    fs = cv2.FileStorage(parameter_path, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_FORMAT_JSON)
    fnode: cv2.FileNode = fs.root()
    detector_params = aruco.DetectorParameters()
    detector_params.readDetectorParameters(fnode)

    camera_0 = CameraTopic("/camera_0/rgb/image_raw", "/camera_0/rgb/camera_info", "/camera_0/tag_detections")
    camera_1 = CameraTopic(
        "/camera_1/image_rect", "/camera_1/camera_info", "/camera_1/tag_detections", is_rectified=True
    )

    board_config = BoardConfig.from_file(board_config_path)
    config = ExstrinsicCalibrationConfig(detector_params=detector_params, board=board_config)
    detector = aruco.CharucoDetector(board_config.board, config.charuco_params, detector_params, config.refine_params)

    app = AppData(camera_0=camera_0, camera_1=camera_1, detector=detector, config=config)

    if not is_live and bag_path is None:
        raise ValueError("Must provide either a bag file or the live flag")
    if is_live:
        compute_calibration_live(app)
    else:
        compute_calibration_from_bag(bag_path, app)


if __name__ == "__main__":
    main()
