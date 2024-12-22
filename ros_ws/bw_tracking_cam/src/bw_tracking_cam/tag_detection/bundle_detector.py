from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
from bw_shared.configs.tag_config import BundleConfig
from bw_shared.geometry.transform3d import Transform3D
from sensor_msgs.msg import CameraInfo

from bw_tracking_cam.tag_detection.tag_detection import TagDetection


def compute_pose_ransac(
    camera_info: CameraInfo, microsac_params: cv2.UsacParams, image_points: np.ndarray, object_points: np.ndarray
) -> Optional[Transform3D]:
    camera_matrix = np.array(camera_info.K).reshape(3, 3)
    dist_coeffs = np.array(camera_info.D)
    (
        success,
        _,
        rot_camera_to_bundle,
        tl_camera_to_bundle,
        inliers,
    ) = cv2.solvePnPRansac(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        params=microsac_params,
    )
    if not success:
        return None
    orientation, _ = cv2.Rodrigues(rot_camera_to_bundle)

    pose = np.eye(4)
    pose[:3, :3] = orientation
    pose[:3, 3] = np.array(tl_camera_to_bundle).flatten()
    return Transform3D(pose)


def compute_pose_pnp(
    camera_info: CameraInfo, image_points: np.ndarray, object_points: np.ndarray
) -> Optional[Transform3D]:
    camera_matrix = np.array(camera_info.K).reshape(3, 3)
    dist_coeffs = np.array(camera_info.D)
    success, rot_camera_to_bundle, tl_camera_to_bundle = cv2.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs
    )
    if not success:
        return None
    orientation, _ = cv2.Rodrigues(rot_camera_to_bundle)

    pose = np.eye(4)
    pose[:3, :3] = orientation
    pose[:3, 3] = np.array(tl_camera_to_bundle).flatten()
    return Transform3D(pose)


@dataclass
class BundleResult:
    config: BundleConfig
    detections: list[TagDetection] = field(default_factory=list)
    bundle_pose: Optional[Transform3D] = None
    tag_poses: dict[int, Transform3D] = field(default_factory=dict)


class BundleDetectorInterface(ABC):
    def __init__(self, config: BundleConfig) -> None:
        self.ids = [tag.tag_id for tag in config]
        self.object_points = {tag.tag_id: tag.bundle_corners for tag in config}
        self.single_tag_points = {tag.tag_id: tag.tag_corners for tag in config}
        self.config = config

    @abstractmethod
    def compute_pose(self, image_points: np.ndarray, object_points: np.ndarray) -> Optional[Transform3D]:
        pass

    def detect(self, detections: list[TagDetection]) -> BundleResult:
        result = BundleResult(self.config)

        image_points = np.array([], dtype=np.float32)
        object_points = np.array([], dtype=np.float32)
        filtered_detections = []
        for detection in detections:
            if detection.tag_id not in self.ids:
                continue
            filtered_detections.append(detection)
            corners = np.array(detection.corners, dtype=np.float32)
            image_points = np.vstack((image_points, corners)) if image_points.size else corners
            tag_points = self.object_points[detection.tag_id]
            object_points = np.vstack((object_points, tag_points)) if object_points.size else tag_points
            single_tag_points = self.single_tag_points[detection.tag_id]
            single_tag_pose = self.compute_pose(corners, single_tag_points)
            if single_tag_pose is not None:
                result.tag_poses[detection.tag_id] = single_tag_pose

        result.detections = filtered_detections

        if image_points.size == 0 or object_points.size == 0:
            return result
        result.bundle_pose = self.compute_pose(image_points, object_points)
        return result


class PnpBundleDetector(BundleDetectorInterface):
    def __init__(self, config: BundleConfig, camera_info: CameraInfo) -> None:
        self.camera_info = camera_info

        super().__init__(config)

    def compute_pose(self, image_points: np.ndarray, object_points: np.ndarray) -> Optional[Transform3D]:
        return compute_pose_pnp(self.camera_info, image_points, object_points)


class RansacBundleDetector(BundleDetectorInterface):
    def __init__(self, config: BundleConfig, microsac_params: cv2.UsacParams, camera_info: CameraInfo) -> None:
        self.microsac_params = microsac_params
        self.camera_info = camera_info

        super().__init__(config)

    def compute_pose(self, image_points: np.ndarray, object_points: np.ndarray) -> Optional[Transform3D]:
        return compute_pose_ransac(self.camera_info, self.microsac_params, image_points, object_points)
