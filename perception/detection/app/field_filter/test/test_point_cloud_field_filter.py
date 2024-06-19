import numpy as np
from app.field_filter.point_cloud_field_filter import PointCloudFieldFilter
from bw_shared.geometry.projection_math.points_transform import points_transform_by
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Vector3


def test_compute_field_centered_plane(field_filter: PointCloudFieldFilter) -> None:
    x_data = np.linspace(-1, 1, 10)
    y_data = np.linspace(-1, 1, 10)
    xx, yy = np.meshgrid(x_data, y_data)
    zz = np.zeros_like(xx)
    points = np.c_[xx.ravel(), yy.ravel(), zz.ravel()]

    input_transform = Transform3D.from_position_and_rpy(Vector3(0.1, -0.2, 2.5), RPY((2.5, 0.1, 0.768)))
    transformed_points = points_transform_by(points, input_transform.tfmat)

    field_centered_plane, extents, inlier_points = field_filter.compute_field_centered_plane(transformed_points)
    assert len(inlier_points) == len(points)
    assert np.allclose(np.array(extents), np.array([2.0, 2.0]))
    assert field_centered_plane.almost_equal(input_transform), f"{field_centered_plane} != {input_transform}"
