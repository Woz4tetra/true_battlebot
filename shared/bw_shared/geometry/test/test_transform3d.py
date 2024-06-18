import numpy as np
import pytest
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3


def test_constructors() -> None:
    assert Transform3D(np.eye(4)) == Transform3D.identity()
    assert Transform3D.from_position_and_quaternion(
        Vector3(1, 2, 3), Quaternion(0.0, 0.707107, 0.0, 0.707107)
    ).almost_equal(
        Transform3D(
            np.array(
                [
                    [0.0, 0.0, 1.0, 1.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [-1.0, 0.0, 0.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
        )
    )

    assert Transform3D.from_position_and_rpy(Vector3(1, 2, 3), RPY((0.0, np.pi / 2, 0.0))).almost_equal(
        Transform3D(
            np.array(
                [
                    [0.0, 0.0, 1.0, 1.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [-1.0, 0.0, 0.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
        )
    )


def test_almost_equal() -> None:
    assert Transform3D(np.eye(4)).almost_equal(Transform3D(np.eye(4)))
    assert not Transform3D(np.eye(4)).almost_equal(Transform3D(np.eye(4) * 2))
    assert Transform3D(
        np.array(
            [
                [0.0, 0.0, 1.0, 1.0],
                [0.0, 1.0, 0.0, 2.0],
                [-1.0, 0.0, 0.0, 3.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
    ).almost_equal(
        Transform3D(
            np.array(
                [
                    [0.0, 0.0, 1.0 + 1e-6, 1.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [-1.0, 0.0, 0.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
        )
    )
    assert not Transform3D(
        np.array(
            [
                [0.0, 0.0, 1.0, 1.0],
                [0.0, 1.0, 0.0, 2.0],
                [-1.0, 0.0, 0.0, 3.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
    ).almost_equal(
        Transform3D(
            np.array(
                [
                    [0.0, 0.0, 1.0 + 3e-6, 1.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [-1.0, 0.0, 0.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
        )
    )


def test_conversion_constructors() -> None:
    assert Transform3D.from_msg(
        Transform(
            translation=Vector3(1, 2, 3),
            rotation=Quaternion(0.0, 0.707107, 0.0, 0.707107),
        )
    ) == Transform3D.from_position_and_quaternion(Vector3(1, 2, 3), Quaternion(0.0, 0.707107, 0.0, 0.707107))
    assert Transform3D.from_pose_msg(
        Pose(
            position=Vector3(1, 2, 3),
            orientation=Quaternion(0.0, 0.707107, 0.0, 0.707107),
        )
    ) == Transform3D.from_position_and_quaternion(Vector3(1, 2, 3), Quaternion(0.0, 0.707107, 0.0, 0.707107))
    assert Transform3D.from_pose2d(Pose2D(x=1, y=2, theta=3)) == Transform3D.from_position_and_rpy(
        Vector3(1, 2, 0), RPY((0.0, 0.0, 3.0))
    )


def test_properties() -> None:
    transform = Transform3D(
        np.array(
            np.array(
                [
                    [0.0, 0.0, 1.0, 1.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [-1.0, 0.0, 0.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
        )
    )
    assert transform.point == Point(1, 2, 3)
    assert transform.position == Vector3(1, 2, 3)
    assert np.all(transform.position_array == np.array([1, 2, 3]))
    assert np.all(
        transform.rotation_matrix
        == np.array(
            [
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
            ]
        )
    )
    assert transform.x == 1
    assert transform.y == 2
    assert transform.z == 3
    assert np.allclose(
        [transform.quaternion.x, transform.quaternion.y, transform.quaternion.z, transform.quaternion.w],
        np.array([0.0, 0.707107, 0.0, 0.707107]),
    )
    assert transform.rpy == RPY((0.0, np.pi / 2, 0.0))


@pytest.mark.parametrize(
    ("pose", "transform", "expected"),
    (
        (
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(0, 0, 0), RPY((0.0, 0.0, np.pi / 2))),
            Transform3D.from_position_and_rpy(Vector3(0, 1, 0), RPY((0.0, 0.0, np.pi / 2))),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, np.pi / 2))),
            Transform3D.from_position_and_rpy(Vector3(0, 0, 0), RPY((0.0, 0.0, np.pi / 2))),
            Transform3D.from_position_and_rpy(Vector3(0, 1, 0), RPY((0.0, 0.0, np.pi))),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(0, 0, 0), RPY((0.0, 0.0, -np.pi / 2))),
            Transform3D.from_position_and_rpy(Vector3(0, -1, 0), RPY((0.0, 0.0, -np.pi / 2))),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(0, 0, 0), RPY((0.0, -np.pi / 2, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(0, 0, 1), RPY((0.0, -np.pi / 2, 0.0))),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(1, 1, 1), RPY((0.0, 0.0, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(2, 3, 4), RPY((0.0, 0.0, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(3, 4, 5), RPY((0.0, 0.0, 0.0))),
        ),
    ),
)
def test_forward_by(pose: Transform3D, transform: Transform3D, expected: Transform3D) -> None:
    transformed_pose = pose.forward_by(transform)
    assert transformed_pose.almost_equal(expected), f"{transformed_pose} != {expected}"


@pytest.mark.parametrize(
    ("pose", "transform", "expected"),
    (
        (
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(0, 0, 0), RPY((0.0, 0.0, np.pi / 2))),
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, np.pi / 2))),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, np.pi / 2))),
            Transform3D.from_position_and_rpy(Vector3(0, 0, 0), RPY((0.0, 0.0, np.pi / 2))),
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, np.pi))),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(0, 0, 0), RPY((0.0, 0.0, -np.pi / 2))),
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, -np.pi / 2))),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, 0.0, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(0, 0, 0), RPY((0.0, -np.pi / 2, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(1, 0, 0), RPY((0.0, -np.pi / 2, 0.0))),
        ),
        (
            Transform3D.from_position_and_rpy(Vector3(1, 1, 1), RPY((0.0, 0.0, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(2, 3, 4), RPY((0.0, 0.0, 0.0))),
            Transform3D.from_position_and_rpy(Vector3(3, 4, 5), RPY((0.0, 0.0, 0.0))),
        ),
    ),
)
def test_transform_by(pose: Transform3D, transform: Transform3D, expected: Transform3D) -> None:
    transformed_pose = pose.transform_by(transform)
    assert transformed_pose.almost_equal(expected), f"{transformed_pose} != {expected}"
