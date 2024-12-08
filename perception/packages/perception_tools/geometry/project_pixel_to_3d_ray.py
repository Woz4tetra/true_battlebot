import math


def project_pixel_to_3d_ray(
    u: float, v: float, cx: float, cy: float, fx: float, fy: float
) -> tuple[float, float, float]:
    """
    :param uv:        rectified pixel coordinates
    :type uv:         (u, v)

    Ripped from image_geometry/cameramodels.py for use in numba.
    Returns the unit vector which passes from the camera center to through rectified pixel (u, v),
    using the camera :math:`P` matrix.
    This is the inverse of :meth:`project3dToPixel`.
    """
    x = (u - cx) / fx
    y = (v - cy) / fy
    norm = math.sqrt(x * x + y * y + 1)
    x /= norm
    y /= norm
    z = 1.0 / norm
    return (x, y, z)
