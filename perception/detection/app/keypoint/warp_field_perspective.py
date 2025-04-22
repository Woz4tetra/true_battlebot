import cv2
import numpy as np
from bw_shared.geometry.array_conversions import array_to_vector3, xyz_to_array
from bw_shared.messages.field import Field
from image_geometry import PinholeCameraModel


def get_warp_perspective_field(
    image_width: int, field: Field, camera_model: PinholeCameraModel
) -> tuple[np.ndarray | None, tuple[int, int] | None]:
    """
    Get the perspective transform matrix and the warped image size for a given field.
    The function checks if the field corners intersect with the plane in the camera.
    If any corner does not intersect, it returns None for both the transform and the size.
    """

    field_corners = []
    for corner in field.corners_in_camera:
        intersection = field.plane_in_camera.ray_intersection(array_to_vector3(corner))
        if intersection is None:
            return None, None
        field_corners.append(xyz_to_array(intersection))
    if len(field_corners) != 4:
        return None, None

    src_corners = []
    for field_corner in field_corners:
        corner_pixel = camera_model.project3dToPixel(field_corner)
        if np.any(np.isnan(corner_pixel)):
            return None, None
        src_corners.append(corner_pixel)

    field_aspect_ratio = field.size.x / field.size.y
    warped_image_height = int(image_width / field_aspect_ratio)
    dst_corners = np.array(
        [
            [0, 0],
            [image_width, 0],
            [image_width, warped_image_height],
            [0, warped_image_height],
        ],
        dtype=np.float32,
    )
    tf_warp = cv2.getPerspectiveTransform(np.array(src_corners, dtype=np.float32), dst_corners)
    return tf_warp, (image_width, warped_image_height)
