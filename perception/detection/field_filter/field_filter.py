from bw_shared.configs.maps import FieldConfig
from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image
from perception_tools.messages.field_result import FieldResult
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray


class FieldFilter:
    def __init__(self, field_config: FieldConfig) -> None:
        self.field_config = field_config

    def compute_field(
        self, segmentations: SegmentationInstanceArray, depth_image: Image, camera_info: CameraInfo
    ) -> FieldResult:
        pass
