import numpy as np
import open3d as o3d
from perception_tools.messages.point_cloud import PointCloud


def cloud_to_open3d(cloud: PointCloud, max_value: float) -> o3d.geometry.PointCloud:
    delete_points = np.isnan(cloud.points) | ((cloud.points > max_value) | (cloud.points < -max_value))
    keep_rows = ~np.any(delete_points, axis=1)
    points = cloud.points[keep_rows]

    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(points)
    if cloud.colors.size > 0:
        colors = cloud.colors[keep_rows]
        cloud_o3d.colors = o3d.utility.Vector3dVector(colors)

    return cloud_o3d
