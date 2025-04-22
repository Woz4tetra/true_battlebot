import logging

import numpy as np
from bw_interfaces.msg import SegmentationInstanceArray
from bw_shared.enums.label import ModelLabel
from perception_tools.inference.simulated_mask_to_contours import (
    make_simulated_segmentation_color_map,
    segmentation_array_to_contour_map,
    simulated_mask_to_contours,
)
from perception_tools.messages.image import Image
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from perception_tools.rosbridge.ros_publisher import RosPublisher
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Empty


class SimulatedSegmentationManager:
    def __init__(
        self,
        sim_segmentation_image_sub: RosPollSubscriber[RosImage],
        simulated_segmentation_sub: RosPollSubscriber[SegmentationInstanceArray],
        layer_request_pub: RosPublisher[Empty],
    ) -> None:
        self.sim_segmentation_image_sub = sim_segmentation_image_sub
        self.simulated_segmentation_sub = simulated_segmentation_sub
        self.layer_request_pub = layer_request_pub
        self.prev_image: Image | None = None
        self.prev_segmentation: SegmentationInstanceArray | None = None
        self.logger = logging.getLogger(self.__class__.__name__)
        self.color_to_model_label_map: dict[int, ModelLabel] = {}

    def request_segmentation(self) -> None:
        self.logger.debug("Requested segmentation")
        self.layer_request_pub.publish(Empty())

    def get_layer_image(self) -> Image | None:
        if image := self.sim_segmentation_image_sub.receive():
            self.prev_image = Image.from_msg(image)
        return self.prev_image

    def get_segmentation(self) -> SegmentationInstanceArray | None:
        if segmentation := self.simulated_segmentation_sub.receive():
            self.prev_segmentation = segmentation
        return self.prev_segmentation

    def convert_layer_image_to_contour_map(
        self, layer_image: np.ndarray, selected_labels: tuple[ModelLabel, ...] | None = None
    ) -> tuple[dict[ModelLabel, list[np.ndarray]] | None, dict[int, Exception]]:
        if selected_labels is None:
            selected_labels = tuple(ModelLabel)
        if segmentation := self.get_segmentation():
            color_to_model_label_map, skipped_labels = make_simulated_segmentation_color_map(
                segmentation, selected_labels
            )
            self.color_to_model_label_map.update(color_to_model_label_map)
        if len(self.color_to_model_label_map) == 0:
            return None, {}

        segmentation_array, exceptions = simulated_mask_to_contours(
            layer_image, self.color_to_model_label_map, selected_labels
        )
        contour_map = segmentation_array_to_contour_map(segmentation_array)
        return contour_map, exceptions
