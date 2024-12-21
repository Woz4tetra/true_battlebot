import numpy as np
from sensor_msgs.msg import CameraInfo


def project_point_array_to_pixel(points: np.ndarray, info: CameraInfo) -> np.ndarray:
    """
    Vectorized implementation of rs2_project_pixel_to_point.

    Only supports null and brown_conrady distortions.
    See https://github.com/IntelRealSense/librealsense/blob/8ffb17b027e100c2a14fa21f01f97a1921ec1e1b/src/rs.cpp#L3512

    Args:
        np.ndarray: (N, 3) pointcloud in sensor frame
        intrinsics  camera intrinsics

    Raises:
        ValueError: Exception raised for invalid distortion model.

    Returns:

        pixels (np.ndarray): (N, 2) array representing N (x,y) coordinates
    """

    distortion = np.array(info.D)
    fx = info.K[0]
    fy = info.K[4]
    ppx = info.K[2]
    ppy = info.K[5]
    pixels: np.ndarray = points[:, :2] / np.expand_dims(points[:, 2], -1)

    old_pixels = pixels.copy()
    r2 = np.sum(pixels**2, axis=-1)
    f = 1.0 + distortion[0] * r2 + distortion[1] * r2**2 + distortion[4] * r2**3
    if len(distortion) >= 8:
        f /= 1.0 + distortion[5] * r2 + distortion[6] * r2**2 + distortion[7] * r2**3
    # Get the value before applying radial correction
    xy_hadamard = pixels[:, 0] * pixels[:, 1]

    pixels *= np.expand_dims(f, -1)

    pixels[:, 0] += 2 * distortion[2] * xy_hadamard + distortion[3] * (r2 + 2 * old_pixels[:, 0] ** 2)
    pixels[:, 1] += 2 * distortion[3] * xy_hadamard + distortion[2] * (r2 + 2 * old_pixels[:, 1] ** 2)
    pixels[:, 0] = pixels[:, 0] * fx + ppx
    pixels[:, 1] = pixels[:, 1] * fy + ppy
    return pixels
