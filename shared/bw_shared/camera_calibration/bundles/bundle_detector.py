from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CameraInfo

from bw_shared.camera_calibration.detector.detection_results import DetectionResults
from bw_shared.configs.tag_config import BundleConfig, TagConfig
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.transform3d import Transform3D


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
    detection: DetectionResults
    bundle_pose: Optional[Transform3D]
    tag_poses: dict[int, Transform3D] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.config_map = {tag_config.tag_id: tag_config for tag_config in self.config}

    def get_config(self, tag_id: int) -> TagConfig:
        return self.config_map[tag_id]


class BundleDetectorInterface(ABC):
    def __init__(self, config: BundleConfig) -> None:
        self.ids = [tag.tag_id for tag in config]
        self.object_points = np.array([(tag.x, tag.y, tag.z) for tag in config])
        self.config = config

    @abstractmethod
    def compute_pose(self, image_points: np.ndarray, object_points: np.ndarray) -> Optional[Transform3D]:
        pass

    def detect(self, detection: DetectionResults) -> BundleResult:
        bundle_pose = self.compute_pose(detection.image_points, detection.object_points)
        result = BundleResult(self.config, detection, bundle_pose)
        poses_by_id = {}
        if bundle_pose is not None:
            tag_points = points_transform_by(self.object_points, bundle_pose.tfmat)
            poses_by_id = {
                tag_id: Transform3D.from_position_and_quaternion(Vector3(*tag_point), bundle_pose.quaternion)
                for tag_id, tag_point in zip(self.ids, tag_points)
            }
        result.tag_poses = poses_by_id
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
