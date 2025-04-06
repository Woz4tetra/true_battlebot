#!/usr/bin/env python
import rospy
from bw_shared.enums.driver_intent import DriverIntent
from bw_tools.get_param import get_param
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32


class SimpleJoystickRelay:
    def __init__(self):
        # parameters from launch file
        self.axis_left = get_param("~axis_left", 0)
        self.axis_right = get_param("~axis_right", 1)
        self.axis_mode = get_param("~axis_mode", 3)
        self.track_width = get_param("~track_width", 1.0)

        self.scale_left = get_param("~scale_left", 1.0)
        self.scale_right = get_param("~scale_right", 1.0)

        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.driver_intent_pub = rospy.Publisher("driver_intent", Int32, queue_size=10)

        # subscription topics
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joystick_callback, queue_size=10)

        rospy.loginfo("Tank joystick relay is ready!")

    def joystick_callback(self, msg: Joy):
        left = msg.axes[self.axis_left] * self.scale_left
        right = msg.axes[self.axis_right] * self.scale_right

        linear = (left + right) / 2.0
        angular = (right - left) / self.track_width

        # publish cmd_vel
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

        driver_intent = DriverIntent.FOLLOW_ME if msg.axes[self.axis_mode] >= 0.0 else DriverIntent.BACK_AWAY
        self.driver_intent_pub.publish(driver_intent.value)

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("joystick_relay")
    try:
        SimpleJoystickRelay().run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting usb_joystick node")
