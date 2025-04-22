import numpy as np
import pytest
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.transform_to_plane import transform_to_plane
from geometry_msgs.msg import Vector3


def rpy_deg(roll: float, pitch: float, yaw: float) -> RPY:
    return RPY((np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)))


@pytest.mark.parametrize(
    ("transform", "expected_plane_center", "expected_plane_normal"),
    [
        (Transform3D.identity(), np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])),
        (
            Transform3D.from_position_and_rpy(Vector3(1.0, 2.0, 3.0), rpy_deg(0.0, 0.0, 0.0)),
            np.array([1.0, 2.0, 3.0]),
            np.array([0.0, 0.0, 1.0]),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(0.0, 0.0, 0.0), rpy_deg(45.0, 0.0, 0.0)),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, -np.sqrt(2) / 2, np.sqrt(2) / 2]),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(0.0, 0.0, 0.0), rpy_deg(0.0, 45.0, 0.0)),
            np.array([0.0, 0.0, 0.0]),
            np.array([np.sqrt(2) / 2, 0.0, np.sqrt(2) / 2]),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(0.0, 0.0, 0.0), rpy_deg(0.0, 0.0, 45.0)),
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(2.0, 3.0, 1.0), rpy_deg(45.0, 45.0, 45.0)),
            np.array([2.0, 3.0, 1.0]),
            np.array([(1 / 2 + np.sqrt(2) / 4), (-1 / 2 + np.sqrt(2) / 4), 1 / 2]),
        ),
    ],
)
def test_transform_to_plane(
    transform: Transform3D, expected_plane_center: np.ndarray, expected_plane_normal: np.ndarray
) -> None:
    plane_center, plane_normal = transform_to_plane(transform.tfmat)
    assert np.allclose(plane_center, expected_plane_center)
    assert np.allclose(plane_normal, expected_plane_normal)
