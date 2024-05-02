from bw_interfaces.msg import EstimatedObject, SegmentationInstanceArray
from bw_shared.configs.maps_config import MapConfig
from perception_tools.messages.point_cloud import PointCloud

from app.config.field_filter_config.ransac_field_filter_config import RansacFieldFilterConfig
from app.field_filter.field_filter_interface import FieldFilterInterface


class RansacFieldFilter(FieldFilterInterface):
    def __init__(self, map_config: MapConfig, field_filter_config: RansacFieldFilterConfig) -> None:
        self.map_config = map_config
        self.field_filter_config = field_filter_config

    def compute_field(self, segmentations: SegmentationInstanceArray, point_cloud: PointCloud) -> EstimatedObject:
        return EstimatedObject()
