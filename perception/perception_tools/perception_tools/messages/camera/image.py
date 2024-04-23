from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from perception_tools.messages.std_msgs.header import Header
from perception_tools.rosbridge.types import RawRosMessage


@dataclass
class Image:
    header: Header
    data: np.ndarray
    type: str = "sensor_msgs/Image"

    def to_raw(self) -> RawRosMessage:
        pass

    @classmethod
    def from_raw(cls, msg: RawRosMessage) -> Image:
        pass
