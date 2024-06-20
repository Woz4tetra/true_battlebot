import cv2
import numpy as np
from bundle_detector import BundleResult, Detection
from bw_shared.geometry.transform3d import Transform3D
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


def get_optimal_font_scale(text, height):
    for scale in reversed(np.arange(0, 6, 0.1)):
        text_size = cv2.getTextSize(text, fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=scale, thickness=1)
        new_height = text_size[0][1]
        if new_height <= height:
            return scale
    return 1.0


def draw_text(
    image: np.ndarray, text: str, position: tuple[int, int], color: tuple[int, int, int], height: int
) -> np.ndarray:
    scale = get_optimal_font_scale(text, height)
    new_position = (position[0], position[1] + height)
    cv2.putText(image, text, new_position, cv2.FONT_HERSHEY_DUPLEX, scale, color, 1, cv2.LINE_AA)
    return image


def draw_pose(
    image: np.ndarray,
    pose: Transform3D,
    info: CameraInfo,
    length: float,
    color: tuple[int, int, int] = (0, 0, 255),
    line_width: int = 2,
) -> None:
    normal = np.array([0, 0, length, 1])
    normal_in_camera = np.dot(pose.tfmat, normal)

    p0_in_pixels = project_point_to_pixel(pose.tfmat[:3, 3], info)
    p1_in_pixels = project_point_to_pixel(normal_in_camera[:3], info)
    cv2.arrowedLine(
        image,
        tuple(p0_in_pixels.astype(int)),
        tuple(p1_in_pixels.astype(int)),
        color,
        line_width,
    )
    cv2.putText(
        image,
        f"Z: {normal_in_camera[2]:.2f}",
        tuple(p1_in_pixels.astype(int)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2,
    )


def draw_detection(
    info: CameraInfo,
    image: np.ndarray,
    detection: Detection,
    tag_pose: Transform3D,
    tag_size: float,
) -> None:
    (pt_a, pt_b, pt_c, pt_d) = detection.corners
    pt_b = (int(pt_b[0]), int(pt_b[1]))
    pt_c = (int(pt_c[0]), int(pt_c[1]))
    pt_d = (int(pt_d[0]), int(pt_d[1]))
    pt_a = (int(pt_a[0]), int(pt_a[1]))
    # draw the bounding box of the AprilTag detection
    cv2.line(image, pt_a, pt_b, (0, 255, 0), 2)
    cv2.line(image, pt_b, pt_c, (0, 255, 0), 2)
    cv2.line(image, pt_c, pt_d, (0, 255, 0), 2)
    cv2.line(image, pt_d, pt_a, (0, 255, 0), 2)
    # draw the origin (x, y)-coordinates of the AprilTag
    ox = int(pt_c[0])
    oy = int(pt_c[1])
    cv2.circle(image, (ox, oy), 5, (0, 0, 255), -1)
    # draw the tag family on the image
    tag_family = detection.family
    text = f"{tag_family}-{detection.tag_id}"
    cv2.putText(
        image,
        text,
        (pt_a[0], pt_a[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
    )
    draw_pose(image, tag_pose, info, tag_size / 2)


def draw_bundle(info: CameraInfo, image: np.ndarray, bundle: BundleResult) -> np.ndarray:
    draw_image = image.copy()
    for detection in bundle.detections:
        if detection.tag_id not in bundle.tag_poses:
            continue
        draw_detection(
            info,
            draw_image,
            detection,
            bundle.tag_poses[detection.tag_id],
            bundle.config.get_tag(detection.tag_id).tag_size,
        )
    bundle_pose = bundle.bundle_pose
    if bundle_pose is None:
        return draw_image

    smallest_size = min([tag.tag_size for tag in bundle.config.tags])
    draw_pose(draw_image, bundle_pose, info, smallest_size / 2, color=(255, 0, 0), line_width=4)

    return draw_image
