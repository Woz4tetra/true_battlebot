from abc import ABC, abstractmethod

from bw_interfaces.msg import KeypointInstanceArray
from perception_tools.messages.image import Image
from sensor_msgs.msg import CameraInfo


class KeypointInterface(ABC):
    @abstractmethod
    def process_image(self, camera_info: CameraInfo, image: Image) -> tuple[KeypointInstanceArray, Image | None]:
        pass
