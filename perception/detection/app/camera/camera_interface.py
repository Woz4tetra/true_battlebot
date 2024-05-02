from abc import ABC, abstractmethod

from perception_tools.messages.camera_data import CameraData


class CameraInterface(ABC):
    @abstractmethod
    def open(self) -> bool:
        pass

    @abstractmethod
    def poll(self) -> CameraData | None:
        pass

    @abstractmethod
    def close(self) -> None:
        pass
