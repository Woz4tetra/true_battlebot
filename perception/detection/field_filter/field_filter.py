from bw_shared.configs.maps import MapConfig
from config.field_filter_config import FieldFilterConfig
from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image
from perception_tools.messages.field_result import FieldResult
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray


class FieldFilter:
    def __init__(self, map_config: MapConfig, field_filter_config: FieldFilterConfig) -> None:
        self.map_config = map_config
        self.field_filter_config = field_filter_config

    def compute_field(
        self, segmentations: SegmentationInstanceArray, depth_image: Image, camera_info: CameraInfo
    ) -> FieldResult:
        pass
