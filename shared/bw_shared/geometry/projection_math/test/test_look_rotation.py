import numpy as np
import pytest
from bw_shared.geometry.input_modulus import input_modulus
from bw_shared.geometry.projection_math.look_rotation import look_rotation
from bw_shared.geometry.rotation_transforms import euler_from_quaternion


@pytest.mark.parametrize(
    ("forward", "expected_angles"),
    (
        ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.0, 1.0, 0.0), (0.0, 0.0, 90.0)),
        ((0.0, 0.0, 1.0), (-180.0, -90.0, 0.0)),
        ((-1000000.0, 0.0, 1000000.0), (0.0, -45.0, 180.0)),
        ((-10.0, 2.0, -4.0), (0.0, 21.42, 168.69)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.0, -1.0, 0.0), (0.0, 0.0, -90.0)),
        ((0.0, 0.0, -1.0), (180.0, 90.0, 0.0)),
    ),
)
def test_look_rotation_z_up(forward: tuple[float, float, float], expected_angles: tuple[float, float, float]) -> None:
    expected_radians = np.deg2rad(expected_angles)
    rotation = look_rotation(np.array(forward))
    angles = np.array(euler_from_quaternion(rotation))
    delta_angle = input_modulus(angles - expected_radians, -np.pi, np.pi)
    assert np.allclose(delta_angle, 0.0, atol=1e-4), f"Expected {expected_angles}, got {tuple(np.rad2deg(angles))}"
