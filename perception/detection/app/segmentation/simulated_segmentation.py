import logging

import cv2
import numpy as np
from bw_shared.enums.labels import Label
from perception_tools.messages.camera.compressed_image import CompressedImage
from perception_tools.messages.camera.image import Image
from perception_tools.messages.segmentation.contour import Contour
from perception_tools.messages.segmentation.segmentation_instance import SegmentationInstance
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray
from perception_tools.messages.segmentation.uv_keypoint import UVKeypoint
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher

from app.config.segmentation_config.simulated_segmentation_config import SimulatedSegmentationConfig
from app.segmentation.segmentation_interface import SegmentationInterface


class SimulatedSegmentation(SegmentationInterface):
    def __init__(
        self,
        config: SimulatedSegmentationConfig,
        sim_segmentation_image_sub: RosPollSubscriber[CompressedImage],
        segmentation_pub: RosPublisher[SegmentationInstanceArray],
        simulated_segmentation_sub: RosPollSubscriber[SegmentationInstanceArray],
    ) -> None:
        self.logger = logging.getLogger("perception")
        self.separately_friendlies = config.separate_friendlies
        self.debug = config.debug
        self.error_range = config.compression_error_tolerance

        self.simulated_to_real_labels: dict[str, Label] = {
            "Mini bot": Label.FRIENDLY_ROBOT if self.separately_friendlies else Label.ROBOT,
            "Main bot": Label.FRIENDLY_ROBOT if self.separately_friendlies else Label.ROBOT,
            "Enemy bot": Label.ROBOT,
            "Field": Label.FIELD,
            "Referee": Label.REFEREE,
        }
        self.real_model_labels = tuple(Label)
        self.simulated_segmentations: dict[int, Label] = {}

        self.segmentation_pub = segmentation_pub
        self.simulated_segmentation_sub = simulated_segmentation_sub
        self.sim_segmentation_image_sub = sim_segmentation_image_sub

    def process_image(self, rgb_image: Image) -> tuple[SegmentationInstanceArray, Image | None]:
        image = self.sim_segmentation_image_sub.receive()
        if image is None:
            self.logger.warning("No simulated segmentation image received yet")
            return SegmentationInstanceArray(), None
        if segmentation := self.simulated_segmentation_sub.receive():
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
            has_holes = (hierarchy.reshape(-1, 4)[:, 3] >= 0).sum() > 0  # type: ignore
            object_index = object_counts[label]
            segmentation = SegmentationInstance(
                contours=[self.to_contours_msg(contour) for contour in contours],
                score=1.0,
                label=label.value,
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
            contour_msg.points.append(UVKeypoint(x, y))
        contour_msg.area = cv2.contourArea(contours)
        return contour_msg
