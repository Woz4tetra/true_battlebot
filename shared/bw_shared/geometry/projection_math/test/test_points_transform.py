import numpy as np
import pytest
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Quaternion, Vector3


@pytest.mark.parametrize(
    ("points", "transform"),
    (
        (
            np.array(
                [
                    [1, 2, 3],
                ]
            ),
            Transform3D.from_position_and_rpy(Vector3(1, 2, 3), RPY((0.0, np.pi / 2, 0.0))).tfmat,
        ),
        (
            np.array(
                [
                    [1, 2, 3],
                    [4, 5, 6],
                    [7, 8, 9],
                ]
            ),
            Transform3D.from_position_and_rpy(Vector3(1, 2, 3), RPY((0.0, np.pi / 2, 0.0))).tfmat,
        ),
        (
            np.array(
                [
                    [3.77552477, 0.81282688, 1.69433334],
                    [0.37986815, 4.95035327, 3.22373601],
                    [1.76835985, 2.67724639, 0.51993059],
                    [0.06986341, 3.08235887, 0.00876045],
                    [2.05687783, 2.0996347, 1.47169057],
                    [3.1514906, 0.30490111, 3.17871058],
                    [0.56445773, 0.0604252, 1.00970474],
                    [3.68624196, 4.30407626, 0.68688419],
                    [2.89538188, 1.57190285, 0.76236959],
                    [3.55321604, 1.22447725, 0.20895183],
                ]
            ),
            Transform3D.from_position_and_rpy(
                Vector3(0.9561677375350892, 0.32348899479849547, 0.6070890870601551),
                RPY((0.02506454693541696, -0.42611514138874884, 0.03926105702274271)),
            ).tfmat,
        ),
    ),
)
def test_points_transform(points: np.ndarray, transform: np.ndarray) -> None:
    tf3d = Transform3D(transform)
    expected_points = []
    for point in points:
        expected_points.append(
            tf3d.transform_by(Transform3D.from_position_and_quaternion(Vector3(*point), Quaternion(w=1))).position_array
        )
    expected_points_array = np.array(expected_points)
    transformed_points = points_transform_by(points, transform)
    assert np.allclose(transformed_points, expected_points_array)
