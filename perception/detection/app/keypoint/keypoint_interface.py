from abc import ABC, abstractmethod

from bw_interfaces.msg import KeypointInstanceArray
from perception_tools.messages.image import Image


class KeypointInterface(ABC):
    @abstractmethod
    def process_image(self, msg: Image) -> tuple[KeypointInstanceArray, Image | None]:
        pass
