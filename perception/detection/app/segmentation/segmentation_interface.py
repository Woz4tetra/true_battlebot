from abc import ABC, abstractmethod

from perception_tools.messages.camera.image import Image
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray


class SegmentationInterface(ABC):
    @abstractmethod
    def process_image(self, msg: Image) -> tuple[SegmentationInstanceArray, Image | None]:
        pass
