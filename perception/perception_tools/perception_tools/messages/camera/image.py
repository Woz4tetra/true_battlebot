from dataclasses import dataclass

import numpy as np
from perception_tools.messages.std_msgs.header import Header


@dataclass
class Image:
    header: Header
    data: np.ndarray
