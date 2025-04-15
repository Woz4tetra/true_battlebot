import logging
import math
import time

from app.config.field_filter.simulated_field_filter_config import SimulatedFieldFilterConfig
from app.field_filter.field_filter_interface import FieldFilterInterface
from app.field_filter.helpers import get_field
from bw_interfaces.msg import EstimatedObject, SegmentationInstanceArray
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Vector3
from perception_tools.inference.common import msg_to_mask
from perception_tools.messages.point_cloud import PointCloud
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber


class SimulatedFieldFilter(FieldFilterInterface):
    def __init__(
        self,
        field_filter_config: SimulatedFieldFilterConfig,
        simulated_field_result_sub: RosPollSubscriber[EstimatedObject],
    ) -> None:
        self.field_filter_config = field_filter_config
        self.simulated_field_result_sub = simulated_field_result_sub
        self.last_field_result: EstimatedObject | None = None
        self.logger = logging.getLogger(self.__class__.__name__)

    def get_filtered_point_cloud(
        self, segmentations: SegmentationInstanceArray, point_cloud: PointCloud
    ) -> PointCloud | None:
        try:
            field = get_field(segmentations)
        except ValueError as e:
            self.logger.debug(f"Failed to get field segmentation: {e}")
            return None

        self.logger.debug("Filtering point cloud with simulated field result.")
        largest_area_contour = max(field.contours, key=lambda x: x.area)
        mask = msg_to_mask(largest_area_contour, segmentations.width, segmentations.height).astype(bool)

        if len(point_cloud.points) == 0:
            self.logger.warning("Point cloud is empty. Skipping filtering.")
            return None

        cloud_size = point_cloud.points.shape[0] * point_cloud.points.shape[1]
        if mask.size != cloud_size:
            self.logger.error(
                f"Mask size {mask.size} does not match point cloud size {cloud_size}. Skipping filtering."
            )
            return None

        self.logger.debug(
            f"Applying mask to point cloud. Mask shape is {mask.shape}. Point cloud shape is {point_cloud.points.shape}"
        )
        # assumes the point cloud is same shape as the contour source image
        filtered_point_cloud = PointCloud(header=point_cloud.header, points=point_cloud.masked_points(mask))

        self.logger.debug(f"Finished filtering point cloud. Shape is {filtered_point_cloud.points.shape}")

        return filtered_point_cloud

    def process_simulated_field(self, field: EstimatedObject) -> None:
        tf_camera_from_field = Transform3D.from_pose_msg(field.pose.pose)
        tf_field_from_camera = tf_camera_from_field.inverse()
        field_yaw = tf_field_from_camera.rpy[2]
        if field_yaw < 0:
            rotate_field = math.pi / 2
        else:
            rotate_field = -1 * math.pi / 2

        tf_fieldrotated_from_camera = tf_field_from_camera.transform_by(
            Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, rotate_field)))
        )
        tf_camera_from_fieldrotated = tf_fieldrotated_from_camera.inverse()
        field.pose.pose = tf_camera_from_fieldrotated.to_pose_msg()

    def compute_field(
        self, segmentations: SegmentationInstanceArray, point_cloud: PointCloud
    ) -> tuple[EstimatedObject | None, PointCloud | None]:
        if not self.last_field_result:
            self.logger.warning("Waiting for simulated field result")
            while not self.last_field_result:
                if result := self.simulated_field_result_sub.receive():
                    self.logger.debug("Got new simulated field result while waiting")
                    self.process_simulated_field(result)
                    self.last_field_result = result
                time.sleep(0.1)
        elif result := self.simulated_field_result_sub.receive():
            self.logger.debug("Got new simulated field result")
            self.last_field_result = result
        self.logger.info(f"Received simulated field result: {result}")

        filtered_point_cloud = self.get_filtered_point_cloud(segmentations, point_cloud)

        return self.last_field_result, filtered_point_cloud
