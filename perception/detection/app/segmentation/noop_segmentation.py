from app.config.segmentation_config.noop_segmentation_config import NoopSegmentationConfig
from app.segmentation.segmentation_interface import SegmentationInterface
from bw_interfaces.msg import LabelMap, SegmentationInstanceArray
from perception_tools.messages.image import Image


class NoopSegmentation(SegmentationInterface):
    def __init__(self, config: NoopSegmentationConfig) -> None:
        pass

    def process_image(self, msg: Image) -> tuple[SegmentationInstanceArray | None, Image | None]:
        return SegmentationInstanceArray(), msg

    def get_model_to_system_labels(self) -> LabelMap:
        return LabelMap()
