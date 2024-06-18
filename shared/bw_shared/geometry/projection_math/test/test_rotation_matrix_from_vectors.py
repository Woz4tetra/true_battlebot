import numpy as np
import pytest
from bw_shared.geometry.projection_math.rotation_matrix_from_vectors import rotation_matrix_from_vectors
from bw_shared.geometry.rotation_transforms import euler_from_matrix, euler_matrix


def euler_matrix_3x3(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Return rotation matrix from Euler angles.
    """
    return euler_matrix(roll, pitch, yaw)[:3, :3]


@pytest.mark.parametrize(
    ("vec1", "vec2", "expected_rotation"),
    (
        (
            np.array([1, 0, 0]),
            np.array([0, 1, 0]),
            euler_matrix_3x3(0.0, 0.0, np.pi / 2),
        ),
        (
            np.array([1, 0, 0]),
            np.array([0, -1, 0]),
            euler_matrix_3x3(0.0, 0.0, -np.pi / 2),
        ),
        (
            np.array([1, 0, 0]),
            np.array([0, 0, 1]),
            euler_matrix_3x3(0.0, -np.pi / 2, 0.0),
        ),
        (
            np.array([1, 0, 0]),
            np.array([0, 1, 1]),
            euler_matrix_3x3(-np.pi / 4, -np.pi / 4, np.pi / 2),
        ),
    ),
)
def test_rotation_matrix_from_vectors(vec1: np.ndarray, vec2: np.ndarray, expected_rotation: np.ndarray) -> None:
    output_rotation = rotation_matrix_from_vectors(vec1, vec2)
    assert np.allclose(
        output_rotation, expected_rotation
    ), f"{vec1} -> {vec2} returned angles {euler_from_matrix(output_rotation)}"


@pytest.mark.parametrize(
    ("vec1", "vec2"),
    (
        (np.array([2, 3, 2.5]), np.array([-3, 1, -3.4])),
        (np.array([1, 6, 4.42]), np.array([-6.5, 4.93, -1.2])),
        (np.array([1, 0, 0]), np.array([1, 0, 0])),
        (np.array([0, 1, 0]), np.array([1, 0, 0])),
    ),
)
def test_rotate_vector_to_another(vec1: np.ndarray, vec2: np.ndarray) -> None:
    mat = rotation_matrix_from_vectors(vec1, vec2)
    vec1_rot = mat.dot(vec1)
    assert np.allclose(vec1_rot / np.linalg.norm(vec1_rot), vec2 / np.linalg.norm(vec2))
