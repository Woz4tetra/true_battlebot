from typing import Protocol

from perception_tools.messages.camera.camera_data import CameraData


class CameraInterface(Protocol):
    def poll(self) -> CameraData | None:
        pass
