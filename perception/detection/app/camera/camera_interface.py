from abc import ABC, abstractmethod

from bw_shared.enums.enum_auto_lower import EnumAutoLowerStr, auto
from perception_tools.messages.camera_data import CameraData


class CameraMode(EnumAutoLowerStr):
    FIELD_FINDER = auto()
    ROBOT_FINDER = auto()


class CameraInterface(ABC):
    @abstractmethod
    def open(self, mode: CameraMode) -> bool:
        pass

    @abstractmethod
    def poll(self) -> CameraData | None:
        pass

    @abstractmethod
    def close(self) -> None:
        pass
