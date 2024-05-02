from abc import ABC, abstractmethod

from bw_interfaces.msg import EstimatedObject, SegmentationInstanceArray
from perception_tools.messages.point_cloud import PointCloud


class FieldFilterInterface(ABC):
    @abstractmethod
    def compute_field(self, segmentations: SegmentationInstanceArray, point_cloud: PointCloud) -> EstimatedObject:
        pass
