from abc import ABC, abstractmethod

from bw_interfaces.msg import KeypointInstanceArray, LabelMap
from bw_shared.messages.field import Field
from perception_tools.messages.image import Image
from sensor_msgs.msg import CameraInfo


class KeypointInterface(ABC):
    @abstractmethod
    def process_image(
        self, camera_info: CameraInfo, image: Image, field: Field
    ) -> tuple[KeypointInstanceArray | None, Image | None]:
        pass

    @abstractmethod
    def get_model_to_system_labels(self) -> LabelMap:
        pass
