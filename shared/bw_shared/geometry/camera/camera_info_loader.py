from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import toml
from sensor_msgs.msg import CameraInfo

from bw_shared.messages.dataclass_utils import from_dict, to_dict
from bw_shared.messages.header import Header


@dataclass
class CameraInfoData:
    width: int = 0
    height: int = 0
    distortion_model: str = "plumb_bob"
    distortion_coeffs: list[float] = field(default_factory=list)
    camera_matrix: list[float] = field(default_factory=list)
    rectification_matrix: list[float] = field(default_factory=list)
    projection_matrix: list[float] = field(default_factory=list)

    def to_dict(self) -> dict:
        return to_dict(self)

    @classmethod
    def from_dict(cls, data: dict) -> CameraInfoData:
        return from_dict(cls, data)

    def to_msg(self, header: Optional[Header] = None) -> CameraInfo:
        return CameraInfo(
            header=header.to_msg() if header is not None else Header.auto().to_msg(),
            height=self.height,
            width=self.width,
            D=self.distortion_coeffs,
            K=self.camera_matrix,  # type: ignore
            R=self.rectification_matrix,  # type: ignore
            P=self.projection_matrix,  # type: ignore
        )

    @classmethod
    def from_msg(cls, msg: CameraInfo) -> CameraInfoData:
        return CameraInfoData(
            width=msg.width,
            height=msg.height,
            distortion_model=msg.distortion_model,
            distortion_coeffs=list(msg.D),
            camera_matrix=list(msg.K),
            rectification_matrix=list(msg.R),
            projection_matrix=list(msg.P),
        )


def read_calibration(calibration_path: str) -> CameraInfo:
    with open(calibration_path, "r") as file:
        data = toml.load(file)
        return CameraInfoData.from_dict(data).to_msg()
