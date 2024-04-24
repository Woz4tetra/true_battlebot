from abc import ABC, abstractmethod

from perception_tools.messages.camera.camera_data import CameraData


class CameraInterface(ABC):
    @abstractmethod
    def poll(self) -> CameraData | None:
        pass
