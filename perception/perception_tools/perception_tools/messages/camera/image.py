from dataclasses import dataclass

import numpy as np
from perception_tools.messages.std_msgs.header import Header
from rosbridge.types import RawRosMessage


@dataclass
class Image:
    header: Header
    data: np.ndarray

    def to_ros_image(self) -> RawRosMessage:
        pass
