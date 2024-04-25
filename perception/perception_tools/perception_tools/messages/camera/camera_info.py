from __future__ import annotations

from dataclasses import dataclass, field

from perception_tools.messages.std_msgs.header import Header
from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class RegionOfInterest:
    x_offset: int = 0
    y_offset: int = 0
    height: int = 0
    width: int = 0
    do_rectify: bool = False
    type: str = "sensor_msgs/RegionOfInterest"

    def to_raw(self) -> RawRosMessage:
        return {
            "x_offset": self.x_offset,
            "y_offset": self.y_offset,
            "height": self.height,
            "width": self.width,
            "do_rectify": self.do_rectify,
        }

    @classmethod
    def from_raw(cls, raw: RawRosMessage) -> RegionOfInterest:
        return cls(
            x_offset=raw["x_offset"],
            y_offset=raw["y_offset"],
            height=raw["height"],
            width=raw["width"],
            do_rectify=raw["do_rectify"],
        )


@dataclass
class CameraInfo:
    header: Header = field(default_factory=lambda: Header.auto())
    height: int = 0
    width: int = 0
    distortion_model: str = ""
    D: list[float] = field(default_factory=list)
    K: list[float] = field(default_factory=list)
    R: list[float] = field(default_factory=list)
    P: list[float] = field(default_factory=list)
    binning_x: int = 0
    binning_y: int = 0
    roi: RegionOfInterest = field(default_factory=RegionOfInterest)
    type: str = "sensor_msgs/CameraInfo"

    def to_raw(self) -> RawRosMessage:
        return {
            "header": self.header.to_raw(),
            "height": self.height,
            "width": self.width,
            "distortion_model": self.distortion_model,
            "D": self.D,
            "K": self.K,
            "R": self.R,
            "P": self.P,
            "binning_x": self.binning_x,
            "binning_y": self.binning_y,
            "roi": self.roi.to_raw(),
        }

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> CameraInfo:
        return cls(
            header=Header.from_raw(msg["header"]),
            height=msg["height"],
            width=msg["width"],
            distortion_model=msg["distortion_model"],
            D=msg["D"],
            K=msg["K"],
            R=msg["R"],
            P=msg["P"],
            binning_x=msg["binning_x"],
            binning_y=msg["binning_y"],
            roi=RegionOfInterest(**msg["roi"]),
        )
