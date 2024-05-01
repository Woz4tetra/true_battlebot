from bw_shared.configs.maps_config import MapConfig
from perception_tools.messages.camera.point_cloud import PointCloud
from perception_tools.messages.field_result import FieldResult
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray

from app.config.field_filter_config.ransac_field_filter_config import RansacFieldFilterConfig
from app.field_filter.field_filter_interface import FieldFilterInterface


class RansacFieldFilter(FieldFilterInterface):
    def __init__(self, map_config: MapConfig, field_filter_config: RansacFieldFilterConfig) -> None:
        self.map_config = map_config
        self.field_filter_config = field_filter_config

    def compute_field(self, segmentations: SegmentationInstanceArray, point_cloud: PointCloud) -> FieldResult:
        return FieldResult()
