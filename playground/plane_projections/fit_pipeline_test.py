import pickle

import numpy as np
from bw_interfaces.msg import SegmentationInstance
from bw_shared.geometry.projection_math.find_minimum_rectangle import (
    find_minimum_rectangle,
    get_rectangle_angle,
    get_rectangle_extents,
)
from bw_shared.geometry.projection_math.points_transform import points_transform
from bw_shared.geometry.projection_math.project_segmentation import project_segmentation, raycast_segmentation
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Vector3
from image_geometry import PinholeCameraModel
from matplotlib import pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform
from zed_interfaces.msg import PlaneStamped

with open("camera_info.pkl", "rb") as file:
    camera_info = pickle.load(file)

with open("field_segmentation.pkl", "rb") as file:
    field_segmentation: SegmentationInstance = pickle.load(file)

with open("plane.pkl", "rb") as file:
    plane: PlaneStamped = pickle.load(file)

# with open("plane_pose.pkl", "rb") as file:
#     plane_pose = pickle.load(file)

with open("transform.pkl", "rb") as file:
    transform = pickle.load(file)


class Arrow3D(FancyArrowPatch):
    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))

        return np.min(zs)


def draw_frame(axis):
    arrow_prop_dict = dict(mutation_scale=20, arrowstyle="->", shrinkA=0, shrinkB=0)

    a = Arrow3D(0, 0, 0, 1, 0, 0, **arrow_prop_dict, color="r")
    axis.add_artist(a)
    a = Arrow3D(0, 0, 0, 0, 1, 0, **arrow_prop_dict, color="g")
    axis.add_artist(a)
    a = Arrow3D(0, 0, 0, 0, 0, 1, **arrow_prop_dict, color="b")
    axis.add_artist(a)


def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def plot_plane(ax, point: np.ndarray, normal: np.ndarray, width: float = 5.0, height: float = 5.0):
    extent_x = width / 2
    extent_y = height / 2
    d = -point.dot(normal)
    xx, yy = np.meshgrid(
        point[0] + np.linspace(-extent_x, extent_x, 10), point[1] + np.linspace(-extent_y, extent_y, 10)
    )
    z = (-normal[0] * xx - normal[1] * yy - d) * 1.0 / normal[2]
    ax.plot_surface(xx, yy, z, alpha=0.2)


def plot_segmentation(axis, segmentation: SegmentationInstance):
    seg_points = []
    for contour in field_segmentation.contours:
        for point in contour.points:
            seg_points.append((point.x, -point.y))
    seg_points = np.array(seg_points)
    axis.plot(seg_points[:, 0], seg_points[:, 1], marker=".")


def plot_plane_mesh(axis, plane, transform: Transform3D = Transform3D.from_position_and_rpy()):
    mesh_points = []
    for mesh_point in plane.mesh.vertices:
        mesh_points.append((mesh_point.x, mesh_point.y, mesh_point.z))
    mesh_points = np.array(mesh_points)
    mesh_points = points_transform(mesh_points, transform.tfmat)
    axis.plot(mesh_points[:, 0], mesh_points[:, 1], mesh_points[:, 2], "g.")


def main():
    fig = plt.figure(1)
    fig.tight_layout()
    fig.set_figwidth(15)
    fig.set_figheight(10)
    axes = [
        plt.subplot(1, 3, 1),
        plt.subplot(1, 3, 2),
        plt.subplot(1, 3, 3, projection="3d"),
    ]
    for axis in axes:
        axis.set_aspect("equal", "box")

    draw_frame(axes[2])

    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)

    field_rotate_tf = Transform3D.from_position_and_rpy(rpy=RPY((0, -np.pi / 2, 0)))

    plane_transform = Transform3D.from_position_and_quaternion(plane.pose.translation, plane.pose.rotation)
    plane_transform = plane_transform.transform_by(field_rotate_tf)
    ray_projection_transform = Transform3D.from_position_and_rpy(rpy=RPY((0, -np.pi / 2, np.pi / 2)))
    plane_transform = plane_transform.forward_by(ray_projection_transform)

    plane_center = plane_transform.position_array
    plane_normal = plane_transform.rotation_matrix @ np.array((0, 0, 1))
    # plane_center = np.array((plane.center.x, plane.center.y, plane.center.z))
    # plane_normal = np.array((plane.normal.x, plane.normal.y, plane.normal.z))

    # ray_projection_transform = Transform3D.from_position_and_rpy(rpy=RPY((np.pi / 2, -np.pi, np.pi / 2)))
    # plane_center = ray_projection_transform.rotation_matrix @ plane_center
    # plane_normal = ray_projection_transform.rotation_matrix @ plane_normal

    plot_segmentation(axes[0], segmentation=field_segmentation)
    plot_plane_mesh(axes[2], plane, ray_projection_transform)
    plot_plane(axes[2], plane_center, plane_normal)

    rays = raycast_segmentation(camera_model, field_segmentation)
    # rays = points_transform(rays, ray_projection_transform.tfmat)
    # rays = np.array([[1.0, 0.0, 0.0]])
    projected_points = project_segmentation(rays, plane_center, plane_normal)

    axes[2].scatter3D(rays[:, 0], rays[:, 1], rays[:, 2])
    axes[2].scatter3D(projected_points[:, 0], projected_points[:, 1], projected_points[:, 2])

    flattened_points = points_transform(projected_points, plane_transform.inverse().tfmat)

    flattened_points2d = flattened_points[:, :2]
    min_rect = find_minimum_rectangle(flattened_points2d)
    angle = get_rectangle_angle(min_rect)
    extents = get_rectangle_extents(min_rect)

    centroid = np.mean(min_rect, axis=0)

    min_rect = np.vstack((min_rect, min_rect[0]))
    axes[1].plot(flattened_points2d[:, 0], flattened_points2d[:, 1], ".")
    axes[1].plot(min_rect[:, 0], min_rect[:, 1], "r")
    axes[1].plot(centroid[0], centroid[1], "x")

    min_rect = np.hstack((min_rect, np.zeros((min_rect.shape[0], 1))))
    min_rect_reprojected = points_transform(min_rect, plane_transform.tfmat)
    axes[2].plot(min_rect_reprojected[:, 0], min_rect_reprojected[:, 1], min_rect_reprojected[:, 2], "r")

    print("angle:", angle)
    print("extents:", extents)

    field_center_projected = Transform3D.from_position_and_rpy(Vector3(centroid[0], centroid[1], 0.0), RPY((0, 0, 0)))
    field_centered_plane = field_center_projected.transform_by(plane_transform)

    plot_plane(axes[2], field_centered_plane.position_array, field_centered_plane.rotation_matrix @ np.array((0, 0, 1)))

    set_axes_equal(axes[2])

    axes[2].set_xlim3d([-5, 5])
    axes[2].set_ylim3d([-5, 5])
    axes[2].set_zlim3d([-5, 5])

    plt.show()


if __name__ == "__main__":
    main()
