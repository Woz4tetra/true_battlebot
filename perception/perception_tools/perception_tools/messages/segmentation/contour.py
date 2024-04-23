from dataclasses import asdict, dataclass

from perception_tools.messages.segmentation.uv_keypoint import UVKeypoint


@dataclass
class Contour:
    points: list[UVKeypoint]
    area: float

    def to_dict(self):
        return asdict(self)
