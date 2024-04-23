from dataclasses import dataclass

from perception_tools.messages.std_msgs.header import Header


@dataclass
class RegionOfInterest:
    x_offset: int
    y_offset: int
    height: int
    width: int
    do_rectify: bool


@dataclass
class CameraInfo:
    header: Header
    height: int
    width: int
    distortion_model: str
    D: list[float]
    K: list[float]
    R: list[float]
    P: list[float]
    binning_x: int
    binning_y: int
    region_of_interest: RegionOfInterest
