import cv2
import numpy as np
from bw_interfaces.msg import Contour, SegmentationInstance, SegmentationInstanceArray, UVKeypoint
from bw_shared.enums.label import ModelLabel


def make_simulated_segmentation_color_map(msg: SegmentationInstanceArray) -> dict[int, ModelLabel]:
    simulated_segmentations = {}
    for instant in msg.instances:
        try:
            label = ModelLabel(instant.label.lower())
        except ValueError:
            continue
        color = instant.class_index
        simulated_segmentations[color] = label
    return simulated_segmentations


def to_contours_msg(contours: np.ndarray) -> Contour:
    contour_msg = Contour([], 0.0)
    for x, y in contours[:, 0]:
        contour_msg.points.append(UVKeypoint(x, y))
    contour_msg.area = cv2.contourArea(contours)
    return contour_msg


def to_contour_array(contour: Contour) -> np.ndarray:
    return np.array([[point.x, point.y] for point in contour.points], dtype=np.int32).reshape(-1, 1, 2)


def color_i32_to_bgr(color: int) -> tuple[int, int, int]:
    return (color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF


def bridge_gaps(image: np.ndarray, distance: int) -> np.ndarray:
    image = cv2.dilate(image, np.ones((distance, distance), np.uint8), iterations=1)
    return cv2.erode(image, np.ones((distance, distance), np.uint8), iterations=1)


def simulated_mask_to_contours(
    layer_image: np.ndarray,
    simulated_segmentation_color_map: dict[int, ModelLabel],
    model_labels: tuple[ModelLabel, ...],
    error_range: int = -1,
    erode_dilate_size: int = 3,
    debug_image: np.ndarray | None = None,
) -> SegmentationInstanceArray:
    segmentation_array = SegmentationInstanceArray()
    for color, label in simulated_segmentation_color_map.items():
        if label not in model_labels:
            continue
        color_bgr = color_i32_to_bgr(color)
        if error_range < 0:
            mask = np.all(layer_image == color_bgr, axis=2).astype(np.uint8)
        else:
            color_nominal = np.array(color_bgr)
            color_lower = color_nominal - error_range
            color_upper = color_nominal + error_range
            mask = cv2.inRange(layer_image, color_lower, color_upper)
        mask = bridge_gaps(mask, erode_dilate_size)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if hierarchy is None:  # empty mask
            print("empty mask", label)
            continue
        has_holes = bool((hierarchy.reshape(-1, 4)[:, 3] >= 0).sum() > 0)  # type: ignore
        segmentation = SegmentationInstance(
            contours=[to_contours_msg(contour) for contour in contours],
            score=1.0,
            label=label,
            class_index=model_labels.index(label),
            object_index=0,
            has_holes=has_holes,
        )
        segmentation_array.instances.append(segmentation)

        if debug_image is not None:
            debug_image = cv2.drawContours(debug_image, contours, -1, color=color_bgr, thickness=1)

    return segmentation_array


def segmentation_array_to_contour_map(
    segmentation_array: SegmentationInstanceArray,
) -> dict[ModelLabel, list[np.ndarray]]:
    contour_map: dict[ModelLabel, list[np.ndarray]] = {
        ModelLabel(instance.label): [] for instance in segmentation_array.instances
    }
    for instance in segmentation_array.instances:
        for contour in instance.contours:
            contour_map[ModelLabel(instance.label)].append(to_contour_array(contour))

    return contour_map
