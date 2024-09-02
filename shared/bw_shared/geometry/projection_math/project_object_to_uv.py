import numpy as np
from bw_interfaces.msg import Contour, EstimatedObject, UVKeypoint
from geometry_msgs.msg import Point
from image_geometry import PinholeCameraModel
from perception_tools.inference.common import contour_to_msg

from bw_shared.enums.keypoint_name import KeypointName
from bw_shared.geometry.transform3d import Transform3D


class ProjectionError(Exception):
    pass


def point_to_camera_pixel(
    tf_camera_from_robot: Transform3D, pos_robotcenter_to_robotpoint: Point, model: PinholeCameraModel
) -> UVKeypoint:
    pos_robotcenter_to_robotpoint_array = np.array(
        [pos_robotcenter_to_robotpoint.x, pos_robotcenter_to_robotpoint.y, pos_robotcenter_to_robotpoint.z, 1]
    )
    pos_camera_to_robotpoint = np.dot(tf_camera_from_robot.tfmat, pos_robotcenter_to_robotpoint_array)
    if pos_camera_to_robotpoint[2] < 0:
        raise ProjectionError(f"Robot point is behind camera: {pos_camera_to_robotpoint}")
    pixel_robot_point = model.project3dToPixel(pos_camera_to_robotpoint[:3])
    if np.any(np.isnan(pixel_robot_point)):
        raise ProjectionError(f"Robot point is outside camera view: {pos_camera_to_robotpoint} -> {pixel_robot_point}")
    return UVKeypoint(x=pixel_robot_point[0], y=pixel_robot_point[1])


def project_object_to_front_back_uv(robot: EstimatedObject, model: PinholeCameraModel) -> tuple[UVKeypoint, UVKeypoint]:
    tf_camera_from_robot = Transform3D.from_pose_msg(robot.pose.pose)
    front_index = robot.keypoint_names.index(KeypointName.FRONT)
    back_index = robot.keypoint_names.index(KeypointName.BACK)
    pos_robotcenter_to_robotfront = robot.keypoints[front_index].position
    pos_robotcenter_to_robotback = robot.keypoints[back_index].position
    forward_pixel = point_to_camera_pixel(tf_camera_from_robot, pos_robotcenter_to_robotfront, model)
    backward_pixel = point_to_camera_pixel(tf_camera_from_robot, pos_robotcenter_to_robotback, model)
    return forward_pixel, backward_pixel


def project_box_object_to_uv(robot: EstimatedObject, model: PinholeCameraModel) -> Contour:
    tf_camera_from_robot = Transform3D.from_pose_msg(robot.pose.pose)
    width = robot.size.x / 2
    length = robot.size.y / 2
    pose_robotcenter_to_robotcorners = [
        Point(width, length, 0),
        Point(-width, length, 0),
        Point(-width, -length, 0),
        Point(width, -length, 0),
    ]

    contour_list = []
    for corner in pose_robotcenter_to_robotcorners:
        corner_pixel = point_to_camera_pixel(tf_camera_from_robot, corner, model)
        contour_list.append([corner_pixel.x, corner_pixel.y])
    contour_array = [np.array(contour_list).astype(np.float32)]
    return contour_to_msg(contour_array)[0]
