from abc import ABC, abstractmethod

from perception_tools.messages.camera.camera_info import CameraInfo
from perception_tools.messages.camera.image import Image
from perception_tools.messages.field_result import FieldResult
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray


class FieldFilterInterface(ABC):
    @abstractmethod
    def compute_field(
        self, segmentations: SegmentationInstanceArray, depth_image: Image, camera_info: CameraInfo
    ) -> FieldResult:
        pass
