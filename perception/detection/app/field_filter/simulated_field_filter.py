import logging

from bw_shared.configs.maps import MapConfig
from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image
from perception_tools.messages.field_result import FieldResult
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber

from app.config.field_filter_config.simulated_field_filter_config import SimulatedFieldFilterConfig
from app.field_filter.field_filter_interface import FieldFilterInterface


class SimulatedFieldFilter(FieldFilterInterface):
    def __init__(
        self,
        map_config: MapConfig,
        field_filter_config: SimulatedFieldFilterConfig,
        simulated_field_result_sub: RosPollSubscriber[FieldResult],
    ) -> None:
        self.map_config = map_config
        self.field_filter_config = field_filter_config
        self.simulated_field_result_sub = simulated_field_result_sub
        self.last_field_result: FieldResult | None = None
        self.logger = logging.getLogger("perception")

    def compute_field(
        self, segmentations: SegmentationInstanceArray, depth_image: Image, camera_info: CameraInfo
    ) -> FieldResult:
        if not self.last_field_result:
            self.logger.warning("Waiting for simulated field result")
            while not self.last_field_result:
                if result := self.simulated_field_result_sub.receive():
                    self.logger.debug("Got new simulated field result while waiting")
                    self.last_field_result = result
        elif result := self.simulated_field_result_sub.receive():
            self.logger.debug("Got new simulated field result")
            self.last_field_result = result
        self.logger.info(f"Received simulated field result: {result}")
        return self.last_field_result
