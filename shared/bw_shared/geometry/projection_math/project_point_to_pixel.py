import numpy as np
from sensor_msgs.msg import CameraInfo


def project_point_to_pixel(point: np.ndarray, info: CameraInfo) -> np.ndarray:
    src = np.zeros((4, 1), np.float64)
    src[:3, 0] = point
    projection = np.array(info.P).reshape(3, 4)
    dst = np.dot(projection, src)
    if dst[2, 0] == 0:
        return np.array([0.0, 0.0])
    dst /= dst[2, 0]
    result = dst[:2, 0]
    if np.any(np.isnan(result)):
        return np.array([0.0, 0.0])
    return result
