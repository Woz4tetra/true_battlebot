import numpy as np
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.messages.header import Header

from perception_tools.messages.point_cloud import PointCloud


def transform_point_cloud(point_cloud: PointCloud, transform: Transform3D, header: Header) -> PointCloud:
    """
    Transform a point cloud by a given transform.

    Args:
        point_cloud: The point cloud to transform.
        transform: The transform to apply to the point cloud.

    Returns:
        The transformed point cloud.
    """
    padded_points = np.concatenate((point_cloud.points, np.ones((point_cloud.points.shape[0], 1))), axis=1)
    transformed_points = np.tensordot(padded_points, transform.tfmat, axes=(1, 1))[:, 0:3]
    transformed_cloud = PointCloud(
        header=header,
        points=transformed_points,
        colors=point_cloud.colors,
        is_bigendian=point_cloud.is_bigendian,
        is_dense=point_cloud.is_dense,
        color_encoding=point_cloud.color_encoding,
    )
    return transformed_cloud
