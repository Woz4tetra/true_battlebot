from abc import ABC, abstractmethod

from perception_tools.messages.camera.point_cloud import PointCloud
from perception_tools.messages.field_result import FieldResult
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray


class FieldFilterInterface(ABC):
    @abstractmethod
    def compute_field(self, segmentations: SegmentationInstanceArray, point_cloud: PointCloud) -> FieldResult:
        pass
