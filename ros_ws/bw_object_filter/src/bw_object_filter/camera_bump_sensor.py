#!/usr/bin/env python
import numpy as np
import rospy
from bw_interfaces.msg import EstimatedObject
from bw_shared.geometry.rpy import RPY
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.xyz import XYZ
from bw_tools.get_param import get_param
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


def compute_delta_rpy(new_quat: Quaternion, old_quat: Quaternion) -> RPY:
    new_transform = Transform3D.from_position_and_quaternion(Vector3(0, 0, 0), new_quat)
    old_transform = Transform3D.from_position_and_quaternion(Vector3(0, 0, 0), old_quat)
    delta_transform = new_transform.transform_by(old_transform.inverse())
    return delta_transform.rpy


class CameraBumpSensor:
    def __init__(self) -> None:
        self.acceleration_threshold = get_param("~acceleration_threshold", 0.1)  # m/s^2
        self.angular_velocity_threshold = np.deg2rad(get_param("~angular_velocity_threshold", 2.0))  # degrees/s
        self.angle_delta_threshold = np.deg2rad(get_param("~angle_delta_threshold", 1.0))  # degrees
        self.prev_imu = Imu()
        self.is_settled = True
        self.imu_sub = rospy.Subscriber("imu", Imu, self.imu_callback, queue_size=10)
        self.field_sub = rospy.Subscriber("/filter/field", EstimatedObject, self.field_callback, queue_size=1)
        self.settled_pub = rospy.Publisher("is_settled", Bool, queue_size=1, latch=True)

    def imu_callback(self, imu: Imu) -> None:
        if not self.prev_imu.header.frame_id:
            self.prev_imu = imu
            return
        if self.is_settled:
            did_get_bumped = (
                self.did_acceleration_change(imu, self.prev_imu)
                or self.did_angular_velocity_change(imu, self.prev_imu)
                or self.did_orientation_change(imu, self.prev_imu)
            )
            if did_get_bumped:
                self.is_settled = False
                rospy.logwarn("Camera got bumped!")
                self.settled_pub.publish(Bool(False))
        self.prev_imu = imu

    def field_callback(self, field: EstimatedObject) -> None:
        if not field.header.frame_id:
            return
        rospy.loginfo("Camera is settled.")
        self.is_settled = True
        self.settled_pub.publish(Bool(True))

    def did_acceleration_change(self, current_imu: Imu, prev_imu: Imu) -> bool:
        return bool(
            (XYZ.from_msg(current_imu.linear_acceleration) - XYZ.from_msg(prev_imu.linear_acceleration)).magnitude()
            > self.acceleration_threshold
        )

    def did_angular_velocity_change(self, current_imu: Imu, prev_imu: Imu) -> bool:
        return bool(
            (XYZ.from_msg(current_imu.angular_velocity) - XYZ.from_msg(prev_imu.angular_velocity)).magnitude()
            > self.angular_velocity_threshold
        )

    def did_orientation_change(self, current_imu: Imu, prev_imu: Imu) -> bool:
        delta_rpy = compute_delta_rpy(current_imu.orientation, prev_imu.orientation)
        return bool(np.any(np.abs(delta_rpy.to_array()) > self.angle_delta_threshold))

    def run(self) -> None:
        self.settled_pub.publish(Bool(False))
        rospy.spin()


def main() -> None:
    rospy.init_node("camera_bump_sensor", log_level=rospy.DEBUG)
    CameraBumpSensor().run()


if __name__ == "__main__":
    main()
