import pickle

import numpy as np
import tf2_geometry_msgs
from bw_interfaces.msg import Contour, SegmentationInstance, UVKeypoint
from bw_object_filter.field_math.find_minimum_rectangle import (
    find_minimum_rectangle,
    get_rectangle_angle,
    get_rectangle_extents,
)
from bw_object_filter.field_math.points_transform import points_transform
from bw_object_filter.field_math.project_segmentation import project_segmentation
from bw_tools.structs.rpy import RPY
from bw_tools.structs.transform3d import Transform3D
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from image_geometry import PinholeCameraModel
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d

with open("camera_info.pkl", "rb") as file:
    camera_info = pickle.load(file)

with open("field_segmentation.pkl", "rb") as file:
    field_segmentation = pickle.load(file)

with open("plane.pkl", "rb") as file:
    plane = pickle.load(file)

# with open("plane_pose.pkl", "rb") as file:
#     plane_pose = pickle.load(file)

with open("transform.pkl", "rb") as file:
    transform = pickle.load(file)


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


def rotate_field_orientation(
    field_orientation: Quaternion,
    rotate_tf: Transform3D,
) -> Quaternion:
    field_tf = Transform3D.from_position_and_quaternion(Vector3(), field_orientation)
    rotated_field = field_tf.transform_by(rotate_tf)
    return rotated_field.quaternion


def plot_plane(ax, plane_transform: Transform3D, extent: float = 5.0):
    normal = plane_transform.rotation_matrix @ np.array([0, 0, 1])
    point = plane_transform.position_array
    d = -point.dot(normal)
    xx, yy = np.meshgrid(point[0] + np.linspace(-extent, extent, 10), point[1] + np.linspace(-extent, extent, 10))
    z = (-normal[0] * xx - normal[1] * yy - d) * 1.0 / normal[2]
    ax.plot_surface(xx, yy, z, alpha=0.2)


camera_lens_pose = PoseStamped(
    header=plane.header,
    pose=Pose(
        position=Point(plane.pose.translation.x, plane.pose.translation.y, plane.pose.translation.z),
        orientation=plane.pose.rotation,
    ),
)
field_rotate_tf = Transform3D.from_position_and_rpy(Vector3(), RPY((0, np.pi / 2, 0)))

plane_pose = tf2_geometry_msgs.do_transform_pose(camera_lens_pose, transform)
plane_pose.pose.orientation = rotate_field_orientation(plane_pose.pose.orientation, field_rotate_tf)


figuv = plt.figure(1)
figuv.tight_layout()
axuv = figuv.subplots(1, 1)
axuv.set_aspect("equal", "box")

fig2d = plt.figure(2)
fig2d.tight_layout()
ax2d = fig2d.subplots(1, 1)
ax2d.set_aspect("equal", "box")

fig3d = plt.figure(3)
fig3d.tight_layout()
ax3d = plt.axes(projection="3d")


seg_points = []
for contour in field_segmentation.contours:  # type: ignore
    contour: Contour
    for point in contour.points:  # type: ignore
        point: UVKeypoint
        seg_points.append((point.x, -point.y))
seg_points = np.array(seg_points)
axuv.plot(seg_points[:, 0], seg_points[:, 1], marker=".")

camera_model = PinholeCameraModel()
camera_model.fromCameraInfo(camera_info)

plane_transform = Transform3D.from_position_and_quaternion(plane_pose.pose.position, plane_pose.pose.orientation)
projected_points = project_segmentation(camera_model, plane_transform, field_segmentation)
counter_rotate = Transform3D.from_position_and_quaternion(Vector3(), plane_pose.pose.orientation).inverse()
flattened_points = points_transform(projected_points, counter_rotate.tfmat)


plot_plane(ax3d, plane_transform)
ax3d.scatter3D([0], [0], [0], marker="x", color="k")
ax3d.scatter3D(projected_points[:, 0], projected_points[:, 1], projected_points[:, 2])
ax3d.scatter3D(flattened_points[:, 0], flattened_points[:, 1], flattened_points[:, 2])
set_axes_equal(ax3d)

flattened_points2d = flattened_points[:, :2]
min_rect = find_minimum_rectangle(flattened_points2d)
angle = get_rectangle_angle(min_rect)
extents = get_rectangle_extents(min_rect)

min_rect = np.vstack((min_rect, min_rect[0]))
ax2d.plot(flattened_points2d[:, 0], flattened_points2d[:, 1], ".")
ax2d.plot(min_rect[:, 0], min_rect[:, 1], "r")

ax3d.plot(flattened_points2d[:, 0], flattened_points2d[:, 1], ".")
ax3d.plot(min_rect[:, 0], min_rect[:, 1], "r")


print("angle:", angle)
print("extents:", extents)

yawed_plane = Transform3D.from_position_and_rpy(Vector3(), RPY((0, 0, angle))).transform_by(plane_transform)
print(yawed_plane)
plt.show()
