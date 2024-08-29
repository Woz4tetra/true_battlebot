from abc import ABC, abstractmethod

from bw_interfaces.msg import LabelMap, SegmentationInstanceArray
from perception_tools.messages.image import Image


class SegmentationInterface(ABC):
    @abstractmethod
    def process_image(self, msg: Image) -> tuple[SegmentationInstanceArray | None, Image | None]:
        pass

    @abstractmethod
    def get_model_to_system_labels(self) -> LabelMap:
        pass
