from pathlib import Path
from typing import Generator

import cv2
import numpy as np
import tqdm
from app.config.metrics_tool.field_config import FieldConfig
from app.config.metrics_tool.video_filter_config import VideoFilterConfig
from bw_shared.configs.size import Size
from bw_shared.geometry.projection_math.project_object_to_uv import point_to_camera_pixel
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.messages.header import Header
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


def play_video(video_path: Path) -> Generator[Image, None, None]:
    cap = cv2.VideoCapture(str(video_path))
    video_duration = cap.get(cv2.CAP_PROP_FRAME_COUNT) / cap.get(cv2.CAP_PROP_FPS)
    pbar = tqdm.tqdm(total=video_duration, bar_format="{l_bar}{bar}| {n:.2f}/{total:.2f}")
    while cap.isOpened():
        video_time = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        pbar.update(video_time - pbar.n)
        frame_number = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        ret, frame = cap.read()
        if not ret:
            break
        header = Header(stamp=video_time, frame_id="", seq=frame_number)
        yield Image(header=header, data=np.array(frame))
    cap.release()


def load_interesting_video_frames(
    video_path: Path,
    video_filter_config: VideoFilterConfig,
    field_config: FieldConfig,
    field_size: Size,
    tf_map_from_camera: Transform3D,
    camera_info: CameraInfo,
) -> Generator[Image, None, None]:
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)
    background_subtractor = cv2.createBackgroundSubtractorMOG2()
    kernel_size = video_filter_config.dilate_kernel_size
    last_motion_time = 0.0
    for image in play_video(video_path):
        if (
            video_filter_config.min_time_interval > 0.0
            and image.header.stamp - last_motion_time < video_filter_config.min_time_interval
        ):
            continue
        last_motion_time = image.header.stamp
        image.data = cv2.resize(image.data, (camera_info.width, camera_info.height), interpolation=cv2.INTER_LINEAR)
        masked_image = mask_field(image, tf_map_from_camera, field_config, field_size, camera_model)
        if video_filter_config.enable_motion_detection:
            fg_mask = background_subtractor.apply(masked_image.data)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
            fg_mask = cv2.morphologyEx(
                fg_mask, cv2.MORPH_OPEN, kernel, iterations=video_filter_config.dilate_iterations
            )
            contours, hierarchy = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            filtered_contours = []
            for contour in contours:
                if cv2.contourArea(contour) < video_filter_config.contour_min_size:
                    continue
                filtered_contours.append(contour)
            if len(filtered_contours) == 0:
                continue
        yield masked_image
