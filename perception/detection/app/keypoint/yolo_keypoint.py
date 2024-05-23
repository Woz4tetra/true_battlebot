from app.config.keypoint_config.yolo_keypoint_config import YoloKeypointConfig
from app.keypoint.keypoint_interface import KeypointInterface
from bw_interfaces.msg import KeypointInstanceArray
from perception_tools.messages.image import Image


class YoloKeypoint(KeypointInterface):
    def __init__(self, config: YoloKeypointConfig) -> None:
        pass

    def process_image(self, msg: Image) -> tuple[KeypointInstanceArray, Image | None]:
        return KeypointInstanceArray(), msg
