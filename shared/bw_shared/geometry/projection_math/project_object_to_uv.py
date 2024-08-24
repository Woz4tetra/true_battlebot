import numpy as np
from bw_interfaces.msg import EstimatedObject, UVKeypoint
from image_geometry import PinholeCameraModel

from bw_shared.geometry.transform3d import Transform3D


class ProjectionError(Exception):
    pass


def point_to_camera_pixel(
    tf_camera_from_robot: Transform3D, pos_robotcenter_to_robotpoint: np.ndarray, model: PinholeCameraModel
) -> UVKeypoint:
    pos_camera_to_robotpoint = np.dot(tf_camera_from_robot.tfmat, pos_robotcenter_to_robotpoint)
    pixel_robot_point = model.project3dToPixel(pos_camera_to_robotpoint[:3])
    if np.any(np.isnan(pixel_robot_point)):
        raise ProjectionError(f"Robot point is outside camera view: {pos_camera_to_robotpoint} -> {pixel_robot_point}")
    return UVKeypoint(x=pixel_robot_point[0], y=pixel_robot_point[1])


def project_object_to_uv(robot: EstimatedObject, model: PinholeCameraModel) -> tuple[UVKeypoint, UVKeypoint]:
    radius = max(robot.size.x, robot.size.y) / 2
    tf_camera_from_robot = Transform3D.from_pose_msg(robot.pose.pose)
    pos_robotcenter_to_robotfront = np.array([0, radius, 0, 1])
    pos_robotcenter_to_robotback = np.array([0, -radius, 0, 1])
    forward_pixel = point_to_camera_pixel(tf_camera_from_robot, pos_robotcenter_to_robotfront, model)
    backward_pixel = point_to_camera_pixel(tf_camera_from_robot, pos_robotcenter_to_robotback, model)
    return forward_pixel, backward_pixel
