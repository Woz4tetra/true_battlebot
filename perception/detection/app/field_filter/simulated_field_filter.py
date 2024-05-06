import logging

from bw_interfaces.msg import EstimatedObject, SegmentationInstanceArray
from bw_shared.configs.maps_config import MapConfig
from perception_tools.messages.point_cloud import PointCloud
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber

from app.config.field_filter_config.simulated_field_filter_config import SimulatedFieldFilterConfig
from app.field_filter.field_filter_interface import FieldFilterInterface


class SimulatedFieldFilter(FieldFilterInterface):
    def __init__(
        self,
        map_config: MapConfig,
        field_filter_config: SimulatedFieldFilterConfig,
        simulated_field_result_sub: RosPollSubscriber[EstimatedObject],
    ) -> None:
        self.map_config = map_config
        self.field_filter_config = field_filter_config
        self.simulated_field_result_sub = simulated_field_result_sub
        self.last_field_result: EstimatedObject | None = None
        self.logger = logging.getLogger("perception")

    def compute_field(
        self, segmentations: SegmentationInstanceArray, point_cloud: PointCloud
    ) -> tuple[EstimatedObject, PointCloud | None]:
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
        # TODO filter point cloud from result
        return self.last_field_result, point_cloud
