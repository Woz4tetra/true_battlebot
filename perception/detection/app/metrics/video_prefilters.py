from pathlib import Path
from typing import Generator

import cv2
import numpy as np
from app.config.metrics_tool.field_config import FieldConfig
from app.config.metrics_tool.video_filter_config import VideoFilterConfig
from app.metrics.video_player import VideoPlayer
from bw_shared.configs.size import Size
from bw_shared.geometry.projection_math.project_object_to_uv import point_to_camera_pixel
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Point
from image_geometry import PinholeCameraModel
from perception_tools.messages.image import Image
from sensor_msgs.msg import CameraInfo


def mask_field(
    image: Image,
    tf_map_from_camera: Transform3D,
    field_config: FieldConfig,
    field_size: Size,
    camera_model: PinholeCameraModel,
) -> Image:
    """
    Project rectangle defined by field_config onto image using camera_config,
    and return the image with the mask applied.
    """
    tf_camera_from_map = tf_map_from_camera.inverse()
    field_x = field_size.x / 2 + field_config.x_buffer
    field_y = field_size.y / 2 + field_config.y_buffer
    points = [
        Point(field_x, field_y, 0),
        Point(-field_x, field_y, 0),
        Point(-field_x, -field_y, 0),
        Point(field_x, -field_y, 0),
    ]
    polygon = []
    for pos_map_to_fieldcorner in points:
        field_corner_uv = point_to_camera_pixel(tf_camera_from_map, pos_map_to_fieldcorner, camera_model)
        polygon.append([field_corner_uv.x, field_corner_uv.y])
    polygon_array = np.array(polygon, np.int32)
    mask = np.zeros((image.data.shape[0], image.data.shape[1]), np.uint8)
    cv2.fillPoly(mask, [polygon_array], (255,))
    masked_image = cv2.bitwise_and(image.data, image.data, mask=mask)
    return Image(header=image.header, data=masked_image)


def compute_static_background(
    image_iter: Generator[Image, None, None], video_filter_config: VideoFilterConfig
) -> Image:
    images = []
    first_image = next(image_iter)
    start_header = first_image.header
    images.append(first_image.data.astype(np.float32))
    for image in image_iter:
        images.append(image.data.astype(np.float32))
    images = np.array(images)[np.linspace(0, len(images) - 1, video_filter_config.median_window_size).astype(np.int32)]
    median_image = np.median(images, axis=0).astype(np.uint8)
    if start_header is None:
        raise RuntimeError("No images found in video")
    return Image(start_header, median_image)


def iter_masked_images(
    video_path: Path,
    video_filter_config: VideoFilterConfig,
    field_config: FieldConfig,
    field_size: Size,
    tf_map_from_camera: Transform3D,
    camera_info: CameraInfo,
) -> Generator[Image, None, None]:
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)
    last_sample_time = 0.0
    with VideoPlayer(video_path) as player:
        while image := player.next():
            if (
                video_filter_config.min_time_interval > 0.0
                and image.header.stamp - last_sample_time < video_filter_config.min_time_interval
            ):
                continue
            last_sample_time = image.header.stamp
            image.data = cv2.resize(image.data, (camera_info.width, camera_info.height), interpolation=cv2.INTER_LINEAR)
            masked_image = mask_field(image, tf_map_from_camera, field_config, field_size, camera_model)
            yield masked_image


def find_blobs_with_background(
    background: np.ndarray, image: np.ndarray, video_filter_config: VideoFilterConfig
) -> dict[int, list[tuple[int, int]]]:
    diff_image = np.abs(image.astype(np.float32) - background.astype(np.float32))
    diff_color_mask = (diff_image > video_filter_config.blobs_mask_threshold).astype(np.uint8) * 255
    diff_mask = np.bitwise_or.reduce(diff_color_mask, axis=2)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    diff_mask = cv2.dilate(diff_mask, kernel, iterations=3)
    diff_mask = cv2.erode(diff_mask, kernel, iterations=3)

    contours, hierarchy = cv2.findContours(diff_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour_data = []
    for contour in contours:
        contour_area = cv2.contourArea(contour)
        if contour_area < video_filter_config.blob_min_size:
            continue
        contour_data.append((contour, contour_area))
    if len(contour_data) < video_filter_config.max_number_of_blobs:
        print(f"Found fewer blobs that the max number. Found {len(contour_data)}")
    else:
        contour_data.sort(key=lambda data: data[1])
    contours_to_track = [data[0] for data in contour_data[: video_filter_config.max_number_of_blobs]]
    points_to_track = {}
    for object_id, contour in enumerate(contours_to_track):
        contour_image = np.zeros(image.shape[:2])
        cv2.drawContours(contour_image, [contour], -1, (255,), -1)
        blob_points = np.array(np.where(contour_image > 0)).T
        selected_indices = np.round(
            np.linspace(0, len(blob_points) - 1, video_filter_config.num_points_per_blob)
        ).astype(int)
        points_to_track[object_id] = [(px[1], px[0]) for px in blob_points[selected_indices]]

    return points_to_track
