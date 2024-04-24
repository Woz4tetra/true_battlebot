from perception_tools.messages.camera.image import Image
from perception_tools.messages.segmentation.segmentation_instance_array import SegmentationInstanceArray

from app.config.segmentation_config.noop_segmentation_config import NoopSegmentationConfig
from app.segmentation.segmentation_interface import SegmentationInterface


class NoopSegmentation(SegmentationInterface):
    def __init__(self, config: NoopSegmentationConfig) -> None:
        pass

    def process_image(self, msg: Image) -> tuple[SegmentationInstanceArray, Image | None]:
        return SegmentationInstanceArray(), None
