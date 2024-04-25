import numpy as np
from perception_tools.messages.camera.camera_data import CameraData
from perception_tools.messages.camera.camera_info import CameraInfo, RegionOfInterest
from perception_tools.messages.camera.image import Image
from perception_tools.messages.std_msgs.header import Header

from app.camera.camera_interface import CameraInterface
from app.config.camera_config.noop_config import NoopCameraConfig


class NoopCamera(CameraInterface):
    def __init__(self, config: NoopCameraConfig) -> None:
        pass

    def poll(self) -> CameraData | None:
        header = Header.auto()
        return CameraData(
            color_image=Image(header, np.array([])),
            depth_image=Image(header, np.array([])),
            camera_info=CameraInfo(
                header,
                height=0,
                width=0,
                distortion_model="",
                D=[],
                K=[],
                R=[],
                P=[],
                binning_x=0,
                binning_y=0,
                region_of_interest=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
            ),
        )
