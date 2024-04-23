from typing import Protocol

from perception_tools.messages.camera.image import Image
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray


class SegmentationInterface(Protocol):
    def process_image(self, msg: Image) -> tuple[SegmentationInstanceArray, Image | None]:
        pass
