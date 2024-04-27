import copy
import logging
import time
from typing import Protocol

import cv2
import numpy as np
from bw_shared.enums.labels import Label
from perception_tools.messages.camera.image import Image
from perception_tools.messages.segmentation.contour import Contour
from perception_tools.messages.segmentation.segmentation_instance import SegmentationInstance
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray
from perception_tools.messages.segmentation.uv_keypoint import UVKeypoint

from app.config.segmentation_config.simulated_segmentation_config import SimulatedSegmentationConfig
from app.segmentation.segmentation_interface import SegmentationInterface
from app.segmentation.simulated_segmentation_manager import SimulatedSegmentationManager


class SegmentationNoise(Protocol):
    probability: float

    def apply_noise(self, segmentation: SegmentationInstance) -> SegmentationInstance | None: ...


class GenericRobotLabelNoise(SegmentationNoise):
    def __init__(self, probability: float, replacement_label: Label | None, filter_labels: tuple[Label, ...]) -> None:
        self.probability = probability
        self.replacement_label = replacement_label
        self.filter_labels = filter_labels

    def apply_noise(self, segmentation: SegmentationInstance) -> SegmentationInstance | None:
        if self.replacement_label and segmentation.label in self.filter_labels:
            filtered_seg = copy.deepcopy(segmentation)
            filtered_seg.label = self.replacement_label
            return filtered_seg
        return segmentation


class NullMeasurementNoise(SegmentationNoise):
    def __init__(self, probability: float) -> None:
        self.probability = probability

    def apply_noise(self, segmentation: SegmentationInstance) -> SegmentationInstance | None:
        return None


def int_to_flags(value: int, length: int) -> list[bool]:
    return [bool((value >> i) & 1) for i in range(length)]


def contour_center(contour: Contour) -> tuple[int, int]:
    x = sum([point.x for point in contour.points]) // len(contour.points)
    y = sum([point.y for point in contour.points]) // len(contour.points)
    return x, y


def multi_contour_center(contours: list[Contour]) -> tuple[int, int]:
    x = sum([contour_center(contour)[0] for contour in contours]) // len(contours)
    y = sum([contour_center(contour)[1] for contour in contours]) // len(contours)
    return x, y


class NoiseGrid:
    def __init__(self, grid: np.ndarray, noise_generators: list[SegmentationNoise]) -> None:
        self.grid = grid
        self.noise_generators = noise_generators
        self.select_generators = [True for _ in noise_generators]
        self.logger = logging.getLogger("perception")

    def reroll(self) -> None:
        selected_names = []
        for index, generator in enumerate(self.noise_generators):
            self.select_generators[index] = np.random.random() < generator.probability
            if self.select_generators[index]:
                selected_names.append(generator.__class__.__name__)
        self.logger.info(f"Selected noise generators: {selected_names}")

    def _apply_noise_by_index(
        self, noise_index: int, segmentation: SegmentationInstance
    ) -> SegmentationInstance | None:
        filtered_seg = segmentation
        enabled_generators = int_to_flags(noise_index, len(self.noise_generators))
        for index, enabled in enumerate(enabled_generators):
            enabled = enabled and self.select_generators[index]
            if not enabled:
                continue
            filtered_seg = self.noise_generators[index].apply_noise(filtered_seg)
            if filtered_seg is None:
                break
        return filtered_seg

    def apply_noise(self, segmentation_array: SegmentationInstanceArray) -> SegmentationInstanceArray:
        cell_height = segmentation_array.height // self.grid.shape[0]
        cell_width = segmentation_array.width // self.grid.shape[1]
        filtered_array = SegmentationInstanceArray(
            segmentation_array.header, segmentation_array.height, segmentation_array.width
        )
        for segmentation in segmentation_array.instances:
            x, y = multi_contour_center(segmentation.contours)
            cell_x = x // cell_width
            cell_y = y // cell_height
            noise_index = int(self.grid[cell_y, cell_x])
            filtered_seg = segmentation
            if noise_index != 0:
                filtered_seg = self._apply_noise_by_index(noise_index, filtered_seg)
            if filtered_seg is None:
                continue
            filtered_array.instances.append(filtered_seg)
        return filtered_array

    def __str__(self) -> str:
        string = f"{self.__class__.__name__}([\n"
        num_combos = 1 << len(self.noise_generators)
        format_str = "#0%db" % num_combos
        for row in self.grid:
            string += "    [" + (", ".join([format(cell, format_str) for cell in row])) + "],\n"
        string += f"], {self.noise_generators})"
        return string


class SimulatedSegmentation(SegmentationInterface):
    def __init__(
        self,
        config: SimulatedSegmentationConfig,
        segmentation_manager: SimulatedSegmentationManager,
    ) -> None:
        self.config = config
        self.logger = logging.getLogger("perception")
        self.debug = config.debug
        self.error_range = self.config.compression_error_tolerance

        self.simulated_to_real_labels = {
            sim_label: Label(real_label) for sim_label, real_label in self.config.simulated_to_real_labels.items()
        }
        if len(self.simulated_to_real_labels) == 0:
            self.logger.warning("No simulated to real label mapping provided")
        self.logger.info(f"Simulated to real label mapping: {self.simulated_to_real_labels}")
        self.real_model_labels = tuple(Label)
        self.simulated_segmentations: dict[int, Label] = {}
        self.noise_grid: NoiseGrid | None = None
        if self.config.apply_noise:
            np.random.seed(4176)
            generators = [
                GenericRobotLabelNoise(0.3, Label.ROBOT, (Label.FRIENDLY_ROBOT, Label.CONTROLLED_ROBOT)),
                NullMeasurementNoise(0.5),
            ]
            num_combos = 1 << len(generators)
            self.noise_grid = NoiseGrid(np.random.randint(0, num_combos, (7, 7)), generators)
            self.logger.info(f"Noise grid: {self.noise_grid}")
            if self.config.random_sample_interval > 0.0:
                self.noise_grid.reroll()
        self.prev_random_sample_time = time.perf_counter()

        self.segmentation_manager = segmentation_manager

    def process_image(self, rgb_image: Image) -> tuple[SegmentationInstanceArray, Image | None]:
        image = self.segmentation_manager.get_image()
        if image is None:
            self.logger.warning("No simulated segmentation image received yet")
            return SegmentationInstanceArray(), None
        if segmentation := self.segmentation_manager.get_segmentation():
            self.simulated_segmentations.update(self.process_segmentation(segmentation))
        if len(self.simulated_segmentations) == 0:
            self.logger.warning("No simulated segmentation received yet")
            return SegmentationInstanceArray(), None

        segmentation_array = SegmentationInstanceArray()

        if self.debug:
            debug_image = Image.from_other(rgb_image)
        else:
            debug_image = None
        object_counts = {label: 0 for label in self.real_model_labels}
        for color, label in self.simulated_segmentations.items():
            color_rgb = self.color_i32_to_rgb(color)
            color_nominal = np.array(color_rgb)
            color_lower = color_nominal - self.error_range
            color_upper = color_nominal + self.error_range
            mask = cv2.inRange(image.data, color_lower, color_upper)
            mask = self.bridge_gaps(mask, 3)
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if hierarchy is None:  # empty mask
                continue
            has_holes = bool((hierarchy.reshape(-1, 4)[:, 3] >= 0).sum() > 0)  # type: ignore
            object_index = object_counts[label]
            segmentation = SegmentationInstance(
                contours=[self.to_contours_msg(contour) for contour in contours],
                score=1.0,
                label=label,
                class_index=self.real_model_labels.index(label),
                object_index=object_index,
                has_holes=has_holes,
            )
            object_counts[label] += 1
            segmentation_array.instances.append(segmentation)

            if debug_image is not None:
                debug_image.data = cv2.drawContours(debug_image.data, contours, -1, color=color_rgb, thickness=1)

        segmentation_array.header = image.header
        segmentation_array.height = image.data.shape[0]
        segmentation_array.width = image.data.shape[1]

        now = time.perf_counter()
        if self.noise_grid is not None:
            if (
                self.config.random_sample_interval > 0.0
                and now - self.prev_random_sample_time > self.config.random_sample_interval
            ):
                self.noise_grid.reroll()
                self.prev_random_sample_time = now
            segmentation_array = self.noise_grid.apply_noise(segmentation_array)

        return segmentation_array, debug_image

    def process_segmentation(self, msg: SegmentationInstanceArray) -> dict[int, Label]:
        simulated_segmentations = {}
        for instant in msg.instances:
            if instant.label not in self.simulated_to_real_labels:
                continue
            color = instant.class_index
            label = self.simulated_to_real_labels[instant.label]
            simulated_segmentations[color] = label
        return simulated_segmentations

    def color_i32_to_rgb(self, color: int) -> tuple[int, int, int]:
        return color & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF

    def bridge_gaps(self, image: np.ndarray, distance: int) -> np.ndarray:
        image = cv2.dilate(image, np.ones((distance, distance), np.uint8), iterations=1)
        return cv2.erode(image, np.ones((distance, distance), np.uint8), iterations=1)

    def to_contours_msg(self, contours: np.ndarray) -> Contour:
        contour_msg = Contour([], 0.0)
        for x, y in contours[:, 0]:
            contour_msg.points.append(UVKeypoint(int(x), int(y)))
        contour_msg.area = cv2.contourArea(contours)
        return contour_msg
