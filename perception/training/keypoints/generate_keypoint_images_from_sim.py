import argparse
import logging
import random
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from bw_interfaces.msg import EstimatedObject
from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.cage_model import CageModel
from bw_shared.enums.label import ModelLabel
from bw_shared.geometry.projection_math.project_object_to_uv import ProjectionError, project_object_to_front_back_uv
from bw_shared.simulation_control.load_cage_model_sizes import load_cage_model_sizes
from bw_shared.simulation_control.randomized.random_robot_grid import SceneSession, generate_random_robot_grid
from bw_shared.simulation_control.simulation_controller import make_simulation_controller
from image_geometry import PinholeCameraModel
from perception_tools.inference.simulated_mask_to_contours import (
    segmentation_array_to_contour_map,
    simulated_mask_to_contours,
)
from perception_tools.initialize_logger import initialize
from perception_tools.rosbridge.check_connection import check_connection
from perception_tools.rosbridge.wait_for_ros_connection import wait_for_ros_connection
from perception_tools.simulation_control.simulation_robot_topic_sync import RobotDataShapshot, SimulationRobotTopicSync
from perception_tools.training.keypoints_config import load_keypoints_config
from perception_tools.training.yolo_keypoint_dataset import (
    YoloKeypointAnnotation,
    YoloKeypointImage,
    YoloVisibility,
)

MODEL_LABEL_TO_SEGMENTATION_LABEL_MAP = {
    ModelLabel.MR_STABS_MK1: ModelLabel.MINI_BOT,
    ModelLabel.MR_STABS_MK2: ModelLabel.MINI_BOT,
    ModelLabel.MRS_BUFF_MK1: ModelLabel.MAIN_BOT,
    ModelLabel.MRS_BUFF_MK2: ModelLabel.MAIN_BOT,
    ModelLabel.ROBOT: ModelLabel.ROBOT,
    ModelLabel.REFEREE: ModelLabel.REFEREE,
}
SEGMENTATION_LABELS = tuple(set(MODEL_LABEL_TO_SEGMENTATION_LABEL_MAP.values()))


def make_annotation_from_robot(
    robot: EstimatedObject,
    model: PinholeCameraModel,
    contour_map: dict[ModelLabel, list[np.ndarray]],
    image_size: tuple[int, int],
    labels: list[ModelLabel],
) -> YoloKeypointAnnotation | None:
    logger = logging.getLogger("perception")
    width, height = image_size
    label = ModelLabel(robot.label)
    segmentation_label = MODEL_LABEL_TO_SEGMENTATION_LABEL_MAP[label]
    if segmentation_label not in contour_map:
        logger.warning(
            f"Missing contour for label: {segmentation_label} -> {label}. Known labels: {contour_map.keys()}"
        )
        return None
    contours = contour_map[segmentation_label]
    try:
        forward, backward = project_object_to_front_back_uv(robot, model)
    except ProjectionError as e:
        logger.warning(f"Projection error: {e}")
        return None
    if len(contours) == 0:
        logger.warning("No contours found")
        return None

    rectangles = np.array([cv2.boundingRect(contour) for contour in contours])
    top_left = np.min(rectangles[:, :2], axis=0)
    max_dims = np.max(rectangles[:, 2:], axis=0)
    bx, by, bw, bh = np.concatenate([top_left, max_dims])

    bx /= width
    by /= height
    bw /= width
    bh /= height

    keypoints = [(point.x / width, point.y / height, YoloVisibility.LABELED_VISIBLE) for point in (forward, backward)]
    return YoloKeypointAnnotation.from_xywh(bx, by, bw, bh, class_index=labels.index(label), keypoints=keypoints)


def record_image_and_keypoints(output_dir: Path, data_snapshot: RobotDataShapshot, labels: list[ModelLabel]) -> bool:
    logger = logging.getLogger("perception")
    color_to_model_label_map = data_snapshot.color_to_model_label_map
    robot = data_snapshot.robots.robots[0]
    image = data_snapshot.image.data
    layer = data_snapshot.layer.data
    robots = data_snapshot.robots
    model = data_snapshot.model

    filename = f"{time.time():.9f}"
    image_annotation = YoloKeypointImage(image_id=filename)

    height, width = image.shape[:2]
    image_size = (width, height)

    segmentations, exceptions = simulated_mask_to_contours(layer, color_to_model_label_map, SEGMENTATION_LABELS)
    contour_map = segmentation_array_to_contour_map(segmentations)
    for robot in robots.robots:
        annotation = make_annotation_from_robot(robot, model, contour_map, image_size, labels)
        if annotation is None:
            logger.info(f"Skipping annotation. Label: {robot.label}")
            return False
        logger.info(f"Adding annotation for label: {robot.label}")

        image_annotation.labels.append(annotation)

    image_path = str(output_dir / f"{filename}.jpg")
    logger.info(f"Saving image to {image_path}")
    cv2.imwrite(image_path, image)

    with open(output_dir / f"{filename}.txt", "w") as file:
        file.write(image_annotation.to_txt())
    return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=str, help="Path to the configuration file. ex: ./keypoint_names_v1.toml")
    parser.add_argument("num_images", type=int, help="Number of images to generate")
    parser.add_argument("-o", "--output_dir", type=str, default="output")
    args = parser.parse_args()

    scenario_name = "image_synthesis"

    initialize()
    print()  # Start log on a fresh line
    logger = logging.getLogger("perception")
    logger.info("Initializing image synthesis")

    output_dir = Path(args.output_dir)
    config = load_keypoints_config(args.config)
    num_images = args.num_images

    if not output_dir.exists():
        output_dir.mkdir(parents=True)

    scene_session = SceneSession()
    shared_config = SharedConfig.from_files()
    written_images = 0
    cage_sizes = load_cage_model_sizes(shared_config.maps, [CageModel.DRIVE_TEST_BOX, CageModel.NHRL_3LB_CAGE])

    uri = wait_for_ros_connection()
    logger.info(f"Connected to ROS master at {uri}")
    rospy.init_node("generate_images_from_sim", log_level=rospy.DEBUG, disable_signals=True)

    simulation_topic_sync = SimulationRobotTopicSync(SEGMENTATION_LABELS)
    simulation_controller = make_simulation_controller()

    try:
        is_done = False
        while not rospy.is_shutdown() and not is_done:
            # select number of robots and duration. Set camera pose.
            duration = random.uniform(1, 2)

            simulation_config = generate_random_robot_grid(scenario_name, scene_session, cage_sizes, duration)
            simulation_controller.configure_simulation(simulation_config)

            # wait for scenario to finish. Repeat.
            for timer in simulation_controller.wait_for_timer(duration):
                data_snapshot = simulation_topic_sync.get_snapshot()
                if not data_snapshot:
                    continue
                if record_image_and_keypoints(output_dir, data_snapshot, config.labels):
                    written_images += 1
                    if written_images >= num_images:
                        is_done = True
                        break
                if not check_connection("localhost", 11311):
                    raise RuntimeError("Failed to connect to ROS master.")
    finally:
        rospy.signal_shutdown("Exiting")


if __name__ == "__main__":
    main()
