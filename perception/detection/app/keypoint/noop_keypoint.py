from app.config.keypoint_config.noop_keypoint_config import NoopKeypointConfig
from app.keypoint.keypoint_interface import KeypointInterface
from bw_interfaces.msg import KeypointInstanceArray
from perception_tools.messages.image import Image


class NoopKeypoint(KeypointInterface):
    def __init__(self, config: NoopKeypointConfig) -> None:
        pass

    def process_image(self, msg: Image) -> tuple[KeypointInstanceArray, Image | None]:
        return KeypointInstanceArray(), msg
