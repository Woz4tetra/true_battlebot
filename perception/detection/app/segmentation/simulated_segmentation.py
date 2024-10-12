import copy
import logging
import time
from typing import Protocol

import numpy as np
from app.config.segmentation_config.simulated_segmentation_config import SimulatedSegmentationConfig
from app.segmentation.segmentation_interface import SegmentationInterface
from app.segmentation.simulated_segmentation_manager import SimulatedSegmentationManager
from bw_interfaces.msg import Contour, LabelMap, SegmentationInstance, SegmentationInstanceArray
from bw_shared.enums.label import Label, ModelLabel
from perception_tools.inference.simulated_mask_to_contours import (
    make_simulated_segmentation_color_map,
    simulated_mask_to_contours,
)
from perception_tools.messages.image import Image


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


def contour_center(contour: Contour) -> tuple[float, float]:
    x = sum([point.x for point in contour.points]) // len(contour.points)
    y = sum([point.y for point in contour.points]) // len(contour.points)
    return x, y


def multi_contour_center(contours: list[Contour]) -> tuple[float, float]:
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
            cell_x = int(x) // cell_width
            cell_y = int(y) // cell_height
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

        self.model_to_system_labels = self.config.model_to_system_labels.labels
        if len(self.model_to_system_labels) == 0:
            self.logger.warning("No simulated to real label mapping provided")
        self.logger.info(f"Simulated to real label mapping: {self.model_to_system_labels}")
        self.model_labels = tuple(ModelLabel)
        self.system_labels = tuple(Label)
        self.class_indices = self.config.model_to_system_labels.get_class_indices(self.model_labels)
        self.color_to_model_label_map: dict[int, ModelLabel] = {}
        self.noise_grid: NoiseGrid | None = None
        if self.config.apply_noise:
            np.random.seed(4176)
            generators: list[SegmentationNoise] = [
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
        self.logger.info("SimulatedSegmentation initialized")

    def process_image(self, color_image: Image) -> tuple[SegmentationInstanceArray | None, Image | None]:
        self.segmentation_manager.request_segmentation()
        if len(color_image.data) == 0:
            self.logger.warning("Empty image received")
            return None, None
        image = self.segmentation_manager.get_image()
        if image is None:
            return None, None
        if segmentation := self.segmentation_manager.get_segmentation():
            color_to_model_label_map, skipped_labels = make_simulated_segmentation_color_map(
                segmentation, self.model_labels
            )
            self.color_to_model_label_map.update(color_to_model_label_map)
        if len(self.color_to_model_label_map) == 0:
            return None, None

        if self.debug:
            debug_image = Image.from_other(color_image)
        else:
            debug_image = None

        segmentation_array, exceptions = simulated_mask_to_contours(
            image.data,
            self.color_to_model_label_map,
            self.model_labels,
            debug_image=debug_image.data if debug_image is not None else None,
        )

        segmentation_array.header = image.header.to_msg()
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

        object_counts: dict[Label, int] = {}
        for instance in segmentation_array.instances:
            label = self.model_to_system_labels.get(ModelLabel(instance.label))
            if label is None:
                continue
            if label == Label.SKIP:
                continue
            instance.label = label
            instance.class_index = self.class_indices[label]
            instance.object_index = object_counts.setdefault(label, 0)
            object_counts[label] += 1

        return segmentation_array, debug_image

    def get_model_to_system_labels(self) -> LabelMap:
        return self.config.model_to_system_labels.to_msg()
