from collections import deque

import numpy as np
from geometry_msgs.msg import Quaternion


class RollingMedianOrientation:
    def __init__(self, window_size: int = 5) -> None:
        self.window_size = window_size
        self.w_window: deque = deque(maxlen=window_size)
        self.x_window: deque = deque(maxlen=window_size)
        self.y_window: deque = deque(maxlen=window_size)
        self.z_window: deque = deque(maxlen=window_size)

    def update(self, orientation: Quaternion) -> Quaternion:
        self.w_window.append(orientation.w)
        self.x_window.append(orientation.x)
        self.y_window.append(orientation.y)
        self.z_window.append(orientation.z)
        return Quaternion(
            w=float(np.median(self.w_window)),
            x=float(np.median(self.x_window)),
            y=float(np.median(self.y_window)),
            z=float(np.median(self.z_window)),
        )
