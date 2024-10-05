import argparse
import os
from typing import Optional, Tuple

import cv2
import numpy as np
import toml
from apriltag_ros.msg import AprilTagDetectionArray
from bw_shared.camera_calibration.board_config import BoardConfig
from bw_shared.geometry.camera.camera_info_loader import CameraInfoData
from bw_shared.geometry.camera.image_rectifier import ImageRectifier
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.transform3d import Transform3D
from bw_tools.tag_detection.draw_helpers import draw_pose, project_point_array_to_pixel
from cv2 import aruco
from cv_bridge import CvBridge
from rosbag.bag import Bag
from sensor_msgs.msg import CameraInfo, Image
from tqdm import tqdm

BRIDGE = CvBridge()
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))


def load_images(bag: Bag) -> list[tuple[np.ndarray, Optional[AprilTagDetectionArray]]]:
    image_msgs: list[Image] = []
    ground_truth: list[AprilTagDetectionArray] = []
    for topic, msg, timestamp in bag.read_messages():
        type_str = str(type(msg))
        if "sensor_msgs__Image" in type_str:
            image_msgs.append(msg)
        elif "apriltag_ros__AprilTagDetectionArray" in type_str:
            ground_truth.append(msg)

    image_stamps = np.array([msg.header.stamp.to_sec() for msg in image_msgs])

    ground_truth_grouped: list[Optional[AprilTagDetectionArray]] = [None for _ in range(len(image_msgs))]
    for tag_msg in ground_truth:
        closest_index = np.argmin(np.abs(image_stamps - tag_msg.header.stamp.to_sec()))
        ground_truth_grouped[closest_index] = tag_msg
    images = [cv2.cvtColor(BRIDGE.imgmsg_to_cv2(msg), cv2.COLOR_RGB2BGR) for msg in image_msgs]

    return list(zip(images, ground_truth_grouped))


def write_calibration(info: CameraInfo, calibration_path: str) -> None:
    with open(calibration_path, "w") as file:
        toml.dump(CameraInfoData.from_msg(info).to_dict(), file)


def read_calibration(calibration_path: str) -> CameraInfo:
    with open(calibration_path, "r") as file:
        return CameraInfoData.from_dict(toml.load(file)).to_msg()


def compute_camera_info(
    config: BoardConfig, parameter_path: str, images: list[np.ndarray], debug: bool = False
) -> CameraInfo:
    detector_params = aruco.DetectorParameters()
    charuco_params = aruco.CharucoParameters()
    refine_params = aruco.RefineParameters()

    fs = cv2.FileStorage(parameter_path, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_FORMAT_JSON)
    fnode: cv2.FileNode = fs.root()
    detector_params.readDetectorParameters(fnode)

    detector_params.markerBorderBits = config.border_bits

    detector = aruco.CharucoDetector(config.board, charuco_params, detector_params, refine_params)

    board_image = config.generate_image()
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(board_image)
    marker_ids = np.sort(marker_ids.flatten())
    assert np.all(
        np.sort(config.ids) == np.sort(marker_ids)
    ), f"IDs do not match. Expected: {config.ids}, got: {marker_ids}"

    all_object_points = []
    all_image_points = []
    image_size: Optional[Tuple[int, int]] = None
    with tqdm(images) as pbar:
        for image in images:
            if image_size is None:
                image_size = (image.shape[1], image.shape[0])
            else:
                assert image_size == (image.shape[1], image.shape[0]), "All images must have the same size"

            charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)

            if charuco_corners is None or charuco_ids is None:
                print("No charuco corners found. Skipping image.")
                continue
            object_points, image_points = config.board.matchImagePoints(charuco_corners, charuco_ids)
            all_image_points.append(image_points)
            all_object_points.append(object_points)
            debug_image = aruco.drawDetectedCornersCharuco(np.copy(image), charuco_corners, charuco_ids, (0, 255, 0))
            if debug:
                cv2.imshow("aruco", debug_image)
                key = chr(cv2.waitKey(-1) & 0xFF)
                if key == "q":
                    break

            pbar.update(1)

    assert image_size is not None, "No images found."
    ret, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        all_object_points, all_image_points, image_size, None, None
    )

    mean_error = 0
    for index in range(len(all_object_points)):
        imgpoints2, _ = cv2.projectPoints(
            all_object_points[index], rvecs[index], tvecs[index], camera_matrix, distortion
        )
        error = cv2.norm(all_image_points[index], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    print("total error: ", mean_error)

    rectification_matrix = np.eye(3)
    projection = np.zeros((3, 4))
    projection[0:3, 0:3] = camera_matrix
    projection_matrix = projection

    info = CameraInfo(
        width=image_size[0],
        height=image_size[1],
        distortion_model="plumb_bob",
        D=distortion.flatten().tolist(),
        K=camera_matrix.flatten().tolist(),
        R=rectification_matrix.flatten().tolist(),
        P=projection_matrix.flatten().tolist(),
    )
    return info


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("image_bag", type=str, help="path to bag")
    parser.add_argument(
        "board_config",
        type=str,
        nargs="?",
        default=os.path.join(SCRIPT_DIR, "simulation_board.toml"),
        help="path to board config",
    )
    parser.add_argument(
        "-p",
        "--parameters",
        type=str,
        default=os.path.join(SCRIPT_DIR, "calibration_parameters.json"),
        help="calibration parameter file",
    )
    parser.add_argument("-c", "--calibration-path", type=str, default="", help="output directory")
    args = parser.parse_args()
    image_bag = args.image_bag
    parameter_path = args.parameters
    calibration_path = args.calibration_path
    board_config_path = args.board_config

    if len(calibration_path) == 0:
        calibration_path = os.path.splitext(image_bag)[0] + ".toml"

    config = BoardConfig.from_file(board_config_path)

    bag = Bag(image_bag)
    bag_data = load_images(bag)
    bag.close()

    if os.path.isfile(calibration_path):
        info = read_calibration(calibration_path)
    else:
        images = [row[0] for row in bag_data]
        info = compute_camera_info(config, parameter_path, images)
        write_calibration(info, calibration_path)
    print(info)

    show_rectified = False
    rectifier = ImageRectifier(info)
    rectified_info = rectifier.get_rectified_info()

    if show_rectified:
        show_info = rectified_info
    else:
        show_info = info

    grid_in_tag = config.grid_points

    for image, ground_truth in bag_data:
        if ground_truth is None:
            continue
        if show_rectified:
            show_image = rectifier.rectify(image)
        else:
            show_image = np.copy(image)
        points = np.zeros((0, 3))
        for detection in ground_truth.detections:
            tf_camera_from_tag = Transform3D.from_pose_msg(detection.pose.pose.pose)
            grid_points = points_transform_by(grid_in_tag, tf_camera_from_tag.tfmat)
            points = np.concatenate((points, grid_points), axis=0)
            draw_pose(show_image, tf_camera_from_tag, show_info, 0.5)
        if len(points) == 0:
            continue
        tag_pixels = project_point_array_to_pixel(points, show_info)
        tag_pixels = tag_pixels.astype(int)
        for tag_pixel in tag_pixels:
            cv2.circle(show_image, tag_pixel, 3, (255, 0, 0), -1)
        cv2.imshow("reproject", show_image)
        key = chr(cv2.waitKey(-1) & 0xFF)
        if key == "q":
            break


if __name__ == "__main__":
    main()
