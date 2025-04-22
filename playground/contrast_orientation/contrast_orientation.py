import argparse
import logging
import time

import cv2
import numpy as np
import rospy
from bw_shared.enums.label import ModelLabel
from bw_shared.geometry.camera.image_rectifier import ImageRectifier
from bw_shared.geometry.xy import XY
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


def compute_keypoints(image: np.ndarray, contour: np.ndarray) -> tuple[XY, XY] | None:
    mask = np.zeros(image.shape[0:2], dtype=np.uint8)
    mask = cv2.drawContours(mask, [contour], -1, (255,), -1)
    crop = cv2.boundingRect(contour)
    cropped_image = image[crop[1] : crop[1] + crop[3], crop[0] : crop[0] + crop[2]]
    cropped_mask = mask[crop[1] : crop[1] + crop[3], crop[0] : crop[0] + crop[2]]
    # grey = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
    # ksize = 3
    # gradient_x = cv2.Sobel(grey, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=ksize)
    # gradient_y = cv2.Sobel(grey, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=ksize)
    # gradient_x = cv2.convertScaleAbs(gradient_x)
    # gradient_y = cv2.convertScaleAbs(gradient_y)
    # combined = cv2.addWeighted(gradient_x, 0.5, gradient_y, 0.5, 0)
    eroded_mask = cv2.erode(cropped_mask, np.ones((3, 3), np.uint8), iterations=1)
    # combined = cv2.bitwise_and(combined, combined, mask=eroded_mask)
    masked_image = cv2.bitwise_and(cropped_image, cropped_image, mask=eroded_mask)
    gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
    normalized = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX, mask=eroded_mask)
    thresholded = cv2.threshold(normalized, 200, 255, cv2.THRESH_BINARY)[1]
    filtered_contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(filtered_contours) == 0:
        return None
    largest_contour = max(filtered_contours, key=cv2.contourArea)
    approx = cv2.approxPolyDP(largest_contour, 0.07 * cv2.arcLength(largest_contour, True), True)
    if len(approx) != 3:
        return None

    cv2.drawContours(cropped_image, [approx], -1, (0, 255, 0), 3)
    approx_reduced = approx[:, 0]
    lengths = [np.linalg.norm(approx_reduced[(i + 1) % 3] - approx_reduced[i]) for i in range(3)]
    shortest_index = lengths.index(min(lengths))
    shortest_line = approx_reduced[shortest_index], approx_reduced[(shortest_index + 1) % 3]
    cv2.line(
        cropped_image,
        tuple(shortest_line[0]),
        tuple(shortest_line[1]),
        (255, 0, 0),
        2,
    )
    shortest_length_angle = np.arctan2(
        shortest_line[1][1] - shortest_line[0][1],
        shortest_line[1][0] - shortest_line[0][0],
    )
    back_keypoint = XY(
        (shortest_line[0][0] + shortest_line[1][0]) / 2,
        (shortest_line[0][1] + shortest_line[1][1]) / 2,
    )
    front_keypoints = []
    angles = []
    for index in range(len(lengths)):
        front_keypoint = XY(
            (approx_reduced[(index) % 3][0]),
            (approx_reduced[(index) % 3][1]),
        )
        front_keypoints.append(front_keypoint)
        angle = np.arctan2(
            back_keypoint.y - front_keypoint.y,
            back_keypoint.x - front_keypoint.x,
        )
        delta_angle = abs(shortest_length_angle - angle) % np.pi
        angles.append(delta_angle)

    max_angle_index = np.argmax(angles)
    selected_front_keypoint = front_keypoints[max_angle_index]
    cv2.line(
        cropped_image,
        tuple(map(int, selected_front_keypoint)),
        tuple(map(int, back_keypoint)),
        (0, 0, 255),
        2,
    )

    cv2.imshow("contrast_orientation", cropped_image)
    key = chr(cv2.waitKey(1) & 0xFF)
    if key == "q":
        quit()


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
    rectifier = None
    while True:
        time.sleep(0.1)
        data_snapshot = simulation_topic_sync.get_snapshot()
        if not data_snapshot:
            continue
        if rectifier is None:
            new_image_width, new_image_height = data_snapshot.camera_info.width, data_snapshot.camera_info.height
            rectifier = ImageRectifier(data_snapshot.camera_info, new_size=(new_image_width, new_image_height))
        color_to_model_label_map = data_snapshot.color_to_model_label_map
        layer = cv2.cvtColor(data_snapshot.layer.data, cv2.COLOR_BGR2RGB)
        rectified_layer = rectifier.rectify(layer)
        segmentations, exceptions = simulated_mask_to_contours(
            rectified_layer, color_to_model_label_map, SEGMENTATION_LABELS
        )
        contour_map = segmentation_array_to_contour_map(segmentations)
        image = data_snapshot.image.data
        rectified_image = rectifier.rectify(image)

        for label, contours in contour_map.items():
            if label != ModelLabel.MAIN_BOT:
                continue
            for contour in contours:
                keypoints = compute_keypoints(rectified_image, contour)


if __name__ == "__main__":
    main()
