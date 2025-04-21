from perception_tools.fix_rosgraph_logging import fix_rosgraph_logging

fix_rosgraph_logging()

import argparse
import logging
import time

import cv2
import numpy as np
import rospy
from bw_shared.enums.label import ModelLabel
from perception_tools.inference.simulated_mask_to_contours import (
    segmentation_array_to_contour_map,
    simulated_mask_to_contours,
)
from perception_tools.initialize_logger import initialize
from perception_tools.rosbridge.wait_for_ros_connection import wait_for_ros_connection
from perception_tools.simulation_control.simulation_robot_topic_sync import SimulationRobotTopicSync

MODEL_LABEL_TO_SEGMENTATION_LABEL_MAP = {
    ModelLabel.MR_STABS_MK1: ModelLabel.MINI_BOT,
    ModelLabel.MR_STABS_MK2: ModelLabel.MINI_BOT,
    ModelLabel.MRS_BUFF_MK1: ModelLabel.MAIN_BOT,
    ModelLabel.MRS_BUFF_MK2: ModelLabel.MAIN_BOT,
    ModelLabel.ROBOT: ModelLabel.ROBOT,
    ModelLabel.REFEREE: ModelLabel.REFEREE,
}
SEGMENTATION_LABELS = tuple(set(MODEL_LABEL_TO_SEGMENTATION_LABEL_MAP.values()))


def main() -> None:
    parser = argparse.ArgumentParser()
    args = parser.parse_args()

    initialize()
    logger = logging.getLogger("main")

    uri = wait_for_ros_connection()
    logger.info(f"Connected to ROS master at {uri}")
    rospy.init_node("contrast_orientation", log_level=rospy.DEBUG, disable_signals=True)
    window_name = "contrast_orientation"
    cv2.namedWindow(window_name)

    simulation_topic_sync = SimulationRobotTopicSync(SEGMENTATION_LABELS)
    while True:
        time.sleep(0.1)
        data_snapshot = simulation_topic_sync.get_snapshot()
        if not data_snapshot:
            continue
        color_to_model_label_map = data_snapshot.color_to_model_label_map
        layer = cv2.cvtColor(data_snapshot.layer.data, cv2.COLOR_BGR2RGB)
        segmentations, exceptions = simulated_mask_to_contours(layer, color_to_model_label_map, SEGMENTATION_LABELS)
        contour_map = segmentation_array_to_contour_map(segmentations)
        image = data_snapshot.image.data

        for label, contours in contour_map.items():
            for contour in contours:
                image = cv2.drawContours(image, [contour], -1, (0, 255, 0), 3)
        cv2.imshow(window_name, image)
        key = chr(cv2.waitKey(1) & 0xFF)
        if key == "q":
            break


if __name__ == "__main__":
    main()
