import logging
import time

import numpy as np
from bw_interfaces.msg import EstimatedObject, SegmentationInstanceArray
from bw_shared.configs.maps_config import MapConfig
from bw_shared.geometry.transform3d import Transform3D
from perception_tools.inference.common import msg_to_mask
from perception_tools.messages.point_cloud import PointCloud
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber

from app.config.field_filter_config.simulated_field_filter_config import SimulatedFieldFilterConfig
from app.field_filter.field_filter_interface import FieldFilterInterface
from app.field_filter.helpers import get_field


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
        try:
            field = get_field(segmentations)
        except ValueError as e:
            self.logger.error(f"Failed to get field segmentation: {e}")
            return EstimatedObject(), point_cloud

        if not self.last_field_result:
            self.logger.warning("Waiting for simulated field result")
            start_time = time.perf_counter()
            while not self.last_field_result:
                if result := self.simulated_field_result_sub.receive():
                    self.logger.debug("Got new simulated field result while waiting")
                    self.last_field_result = result
                if time.perf_counter() - start_time > 5:
                    self.logger.error("Timed out waiting for simulated field result")
                    return EstimatedObject(), point_cloud
        elif result := self.simulated_field_result_sub.receive():
            self.logger.debug("Got new simulated field result")
            self.last_field_result = result
        self.logger.info(f"Received simulated field result: {result}")

        self.logger.debug("Filtering point cloud with simulated field result.")
        mask = msg_to_mask(field.contours, segmentations.width, segmentations.height).astype(bool)

        if len(point_cloud.points) == 0:
            self.logger.error("Point cloud is empty. Skipping filtering.")
            return EstimatedObject(), point_cloud

        cloud_size = point_cloud.points.shape[0] * point_cloud.points.shape[1]
        if mask.size != cloud_size:
            self.logger.error(
                f"Mask size {mask.size} does not match point cloud size {cloud_size}. Skipping filtering."
            )
            return EstimatedObject(), point_cloud

        self.logger.debug(
            f"Applying mask to point cloud. Mask shape is {mask.shape}. "
            f"Point cloud shape is {point_cloud.points.shape}"
        )
        # assumes the point cloud is same shape as the contour source image
        filtered_point_cloud = PointCloud(header=point_cloud.header, points=point_cloud.masked_points(mask))

        self.logger.debug(f"Finished filtering point cloud. Shape is {filtered_point_cloud.points.shape}")

        plane_transform = Transform3D.from_pose_msg(self.last_field_result.pose.pose)
        self.logger.debug(f"Field centered plane transform: {plane_transform}")

        return self.last_field_result, filtered_point_cloud
