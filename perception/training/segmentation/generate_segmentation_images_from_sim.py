import argparse
import logging
import random
import time
from datetime import datetime
from pathlib import Path

import cv2
import rospy
from bw_shared.configs.shared_config import SharedConfig
from bw_shared.enums.cage_model import CageModel
from bw_shared.enums.label import ModelLabel
from bw_shared.geometry.projection_math.project_object_to_uv import ProjectionError, project_box_object_to_uv
from perception_tools.inference.simulated_mask_to_contours import to_contour_array
from perception_tools.initialize_logger import initialize
from perception_tools.rosbridge.wait_for_ros_connection import wait_for_ros_connection
from perception_tools.simulation_control.load_cage_model_sizes import load_cage_model_sizes
from perception_tools.simulation_control.randomized.random_robot_grid import SceneSession, generate_random_robot_grid
from perception_tools.simulation_control.simulation_controller import make_simulation_controller
from perception_tools.simulation_control.simulation_field_topic_sync import FieldDataShapshot, SimulationFieldTopicSync
from perception_tools.training.coco_dataset import (
    CocoMetaDataset,
    DatasetAnnotation,
    DatasetCategory,
    DatasetImage,
)
from perception_tools.training.instance_helpers import write_dataset

SEGMENTATION_LABELS = (ModelLabel.BACKGROUND, ModelLabel.FIELD)


def record_image_and_segmentation(output_dir: Path, dataset: CocoMetaDataset, data_snapshot: FieldDataShapshot) -> bool:
    logger = logging.getLogger("perception")
    field = data_snapshot.field
    image = data_snapshot.image.data
    model = data_snapshot.model

    filename = f"{time.time():.9f}.jpg"

    try:
        contour_msg = project_box_object_to_uv(field, model)
    except ProjectionError as e:
        logger.warning(f"Projection error: {e}")
        return False
    contour_array = to_contour_array(contour_msg)
    bbox = cv2.boundingRect(contour_array)

    image_path = str(output_dir / filename)
    logger.info(f"Saving image to {image_path}")
    cv2.imwrite(image_path, image)

    dataset_image = DatasetImage(
        id=-1,
        license=0,
        file_name=filename,
        height=image.shape[0],
        width=image.shape[1],
        date_captured=datetime.strftime(datetime.fromtimestamp(data_snapshot.image.header.stamp), "%Y-%m-%d %H:%M:%S"),
    )
    annotation = DatasetAnnotation(
        id=-1,
        image_id=-1,
        category_id=SEGMENTATION_LABELS.index(ModelLabel.FIELD),
        segmentation=[contour_array.flatten().tolist()],
        area=contour_msg.area,
        bbox=list(bbox),
        iscrowd=0,
    )
    dataset.add_annotation(dataset_image, annotations=[annotation])
    return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("num_images", type=int, help="Number of images to generate")
    parser.add_argument("-o", "--output_dir", type=str, default="output")
    args = parser.parse_args()

    scenario_name = "image_synthesis"

    initialize()
    print()  # Start log on a fresh line
    logger = logging.getLogger("perception")
    logger.info("Initializing image synthesis")

    output_dir = Path(args.output_dir)
    num_images = args.num_images

    if not output_dir.exists():
        output_dir.mkdir(parents=True)

    annotations_path = str(output_dir / "_annotations.coco.json")

    dataset = CocoMetaDataset()
    dataset.dataset.categories = [
        DatasetCategory(
            id=index,
            name=label.value,
            supercategory="",
        )
        for index, label in enumerate(SEGMENTATION_LABELS)
    ]
    scene_session = SceneSession()
    shared_config = SharedConfig.from_files()
    written_images = 0
    cage_sizes = load_cage_model_sizes(shared_config.maps, [CageModel.DRIVE_TEST_BOX, CageModel.NHRL_3LB_CAGE])

    uri = wait_for_ros_connection()
    logger.info(f"Connected to ROS master at {uri}")
    rospy.init_node("generate_images_from_sim")

    simulation_topic_sync = SimulationFieldTopicSync()
    simulation_controller = make_simulation_controller()

    rospy.sleep(2.0)  # wait for publishers to connect

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
                if record_image_and_segmentation(output_dir, dataset, data_snapshot):
                    written_images += 1
                    if written_images >= num_images:
                        is_done = True
                        break
                    if written_images % 50 == 0:
                        write_dataset(dataset, annotations_path)
    finally:
        write_dataset(dataset, annotations_path)
        rospy.signal_shutdown("Exiting")


if __name__ == "__main__":
    main()
