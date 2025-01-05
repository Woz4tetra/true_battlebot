from dataclasses import dataclass

import numpy as np
from bw_shared.messages.header import Header


@dataclass
class TrackingData:
    header: Header
    contours: dict[int, list[np.ndarray]]
