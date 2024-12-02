from typing import Optional

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo


def resize_camera_info(camera_info: CameraInfo, dims: tuple[int, int]) -> CameraInfo:
    new_width, new_height = dims
    new_camera_info = CameraInfo()
    scale_y = new_height / camera_info.height
    scale_x = new_width / camera_info.width
    new_camera_info.height = new_height
    new_camera_info.width = new_width
    intrinsics = list(camera_info.K)
    projection = list(camera_info.P)

    intrinsics[0] *= scale_x  # fx
    intrinsics[2] *= scale_x  # cx
    intrinsics[4] *= scale_y  # fy
    intrinsics[5] *= scale_y  # cy
    new_camera_info.K = tuple(intrinsics)  # type: ignore

    projection[0] *= scale_x  # fx
    projection[2] *= scale_x  # cx
    projection[3] *= scale_x  # T
    projection[5] *= scale_y  # fy
    projection[6] *= scale_y  # cy
    new_camera_info.P = tuple(projection)  # type: ignore
    new_camera_info.R = camera_info.R
    new_camera_info.D = camera_info.D

    new_camera_info.roi = camera_info.roi
    new_camera_info.roi.x_offset = int(camera_info.roi.x_offset * scale_x)
    new_camera_info.roi.y_offset = int(camera_info.roi.y_offset * scale_y)
    new_camera_info.roi.width = int(camera_info.roi.width * scale_x)
    new_camera_info.roi.height = int(camera_info.roi.height * scale_y)

    return new_camera_info


class ImageRectifier:
    def __init__(
        self, camera_info: CameraInfo, new_size: Optional[tuple[int, int]] = None, alpha: float = 0.0, padding: int = 0
    ) -> None:
        if new_size is None:
            new_width = camera_info.width
            new_height = camera_info.height
        else:
            new_width, new_height = new_size
        self.unpadded_width = new_width
        self.unpadded_height = new_height
        self.width = new_width + 2 * padding
        self.height = new_height + 2 * padding
        dims = (self.unpadded_width, self.unpadded_height)
        self.camera_info = resize_camera_info(camera_info, dims)
        intrinsics = np.array(self.camera_info.K).reshape(3, 3)
        intrinsics[0, 2] += padding
        intrinsics[1, 2] += padding
        distortion = np.array(self.camera_info.D)
        new_matrix, roi = cv2.getOptimalNewCameraMatrix(
            intrinsics,
            distortion,
            dims,
            alpha,
            dims,
        )
        self.header = self.camera_info.header
        self.new_matrix = np.array(new_matrix)

        # set aspect ratio to original image
        focal_length = max(self.new_matrix[0, 0], self.new_matrix[1, 1])
        self.new_matrix[0, 0] = focal_length
        self.new_matrix[1, 1] = focal_length
        self.new_projection = np.zeros((3, 4))
        self.new_projection[:3, :3] = self.new_matrix

        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            intrinsics,
            distortion,
            np.array([]),
            self.new_matrix,
            dims,
            cv2.CV_32FC1,
        )
        self.padding = padding

    def rectify(self, image: np.ndarray) -> np.ndarray:
        if image.shape[0] != self.unpadded_height or image.shape[1] != self.unpadded_width:
            image = cv2.resize(image, (self.unpadded_width, self.unpadded_height))
        new_image: np.ndarray = cv2.remap(image, self.mapx, self.mapy, cv2.INTER_LINEAR)
        if self.padding > 0:
            new_image = cv2.copyMakeBorder(
                new_image,
                self.padding,
                self.padding,
                self.padding,
                self.padding,
                cv2.BORDER_CONSTANT,
                dst=None,
                value=(0,),
            )
        return new_image

    def get_rectified_info(self) -> CameraInfo:
        return CameraInfo(
            header=self.header,
            width=self.width,
            height=self.height,
            K=self.new_matrix.flatten().tolist(),
            D=[0.0, 0.0, 0.0, 0.0, 0.0],
            P=self.new_projection.flatten().tolist(),
        )
