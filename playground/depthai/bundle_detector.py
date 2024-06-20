from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from cv2 import aruco
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CameraInfo


@dataclass
class Detection:
    family: str
    tag_id: int
    hamming: int
    goodness: float
    decision_margin: float
    homography: np.ndarray
    center: np.ndarray
    corners: np.ndarray


class ArucoDetector:
    def __init__(self) -> None:
        detect_params = aruco.DetectorParameters()
        refine_params = aruco.RefineParameters()
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11),
            detect_params,
            refine_params,
        )

    def detect(self, image: np.ndarray) -> list[Detection]:
        all_corners, ids, rejected = self.detector.detectMarkers(image)
        if ids is None or len(all_corners) == 0:
            return []
        detections = []
        for all_corners, tag_ids in zip(all_corners, ids):
            corners = all_corners[0]
            center = corners.mean(axis=0)
            detections.append(
                Detection(
                    "tag36h11",
                    tag_id=tag_ids[0],
                    hamming=0,
                    goodness=0,
                    decision_margin=0,
                    homography=np.array([]),
                    center=center,
                    corners=corners,
                )
            )
        return detections


@dataclass
class TagConfig:
    tag_id: int
    tag_size: float
    x: float  # meters
    y: float  # meters
    z: float  # meters
    roll: float  # degrees
    pitch: float  # degrees
    yaw: float  # degrees

    @property
    def transform(self) -> Transform3D:
        return Transform3D.from_position_and_rpy(
            Vector3(self.x, self.y, self.z),
            RPY(
                (
                    np.deg2rad(self.roll),
                    np.deg2rad(self.pitch),
                    np.deg2rad(self.yaw),
                )
            ),
        )

    @property
    def tag_corners(self) -> np.ndarray:
        s = self.tag_size / 2
        return np.array(
            [
                [-s, -s, 0.0, 1.0],
                [s, -s, 0.0, 1.0],
                [s, s, 0.0, 1.0],
                [-s, s, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

    @property
    def bundle_corners(self) -> np.ndarray:
        corners_bundle_space = np.dot(self.transform.tfmat, self.tag_corners.T).T
        return corners_bundle_space[:, :3]


@dataclass
class BundleConfig:
    name: str
    tags: list[TagConfig]

    def get_tag(self, tag_id: int) -> TagConfig:
        for tag in self.tags:
            if tag.tag_id == tag_id:
                return tag
        raise ValueError(f"Tag {tag_id} not found in bundle {self.name}")


@dataclass
class BundleResult:
    config: BundleConfig
    detections: list[Detection] = field(default_factory=list)
    bundle_pose: Optional[Transform3D] = None
    tag_poses: dict[int, Transform3D] = field(default_factory=dict)


class BundleDetector:
    def __init__(
        self, config: BundleConfig, detector: ArucoDetector, microsac_params: cv2.UsacParams, camera_info: CameraInfo
    ) -> None:
        self.detector = detector
        self.microsac_params = microsac_params
        self.ids = [tag.tag_id for tag in config.tags]
        self.object_points = {tag.tag_id: tag.bundle_corners for tag in config.tags}
        self.single_tag_points = {tag.tag_id: tag.tag_corners for tag in config.tags}
        self.camera_info = CameraInfo()
        self.config = config
        self.camera_info = camera_info

    def compute_pose(self, image_points: np.ndarray, object_points: np.ndarray) -> Optional[Transform3D]:
        camera_matrix = np.array(self.camera_info.K).reshape(3, 3)
        dist_coeffs = np.array(self.camera_info.D)
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
            params=self.microsac_params,
        )
        # success, rot_camera_to_bundle, tl_camera_to_bundle = cv2.solvePnP(
        #     object_points, image_points, camera_matrix, dist_coeffs
        # )
        if not success:
            return None
        orientation, _ = cv2.Rodrigues(rot_camera_to_bundle)

        pose = np.eye(4)
        pose[:3, :3] = orientation
        pose[:3, 3] = tl_camera_to_bundle.flatten()
        return Transform3D(pose)

    def detect(self, image: np.ndarray) -> BundleResult:
        detections: list[Detection] = self.detector.detect(image)
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
