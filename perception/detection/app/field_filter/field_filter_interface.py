from abc import ABC, abstractmethod

from bw_interfaces.msg import EstimatedObject, SegmentationInstanceArray


class FieldFilterInterface(ABC):
    @abstractmethod
    def compute_field(
        self, segmentations: SegmentationInstanceArray, point_cloud: SegmentationInstanceArray
    ) -> EstimatedObject:
        pass
