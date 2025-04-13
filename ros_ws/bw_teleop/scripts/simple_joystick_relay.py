#!/usr/bin/env python
import rospy
from bw_tools.get_param import get_param
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class SimpleJoystickRelay:
    def __init__(self):
        # parameters from launch file
        self.axis_linear = get_param("~axis_linear", 0)
        self.axis_angular = get_param("~axis_angular", 1)

        self.scale_linear = get_param("~scale_linear", 1.0)
        self.scale_angular = get_param("~scale_angular", 1.0)

        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # subscription topics
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joystick_callback, queue_size=10)

        rospy.loginfo("Simple joystick relay is ready!")

    def joystick_callback(self, msg: Joy):
        linear = msg.axes[self.axis_linear] * self.scale_linear
        angular = msg.axes[self.axis_angular] * self.scale_angular

        # publish cmd_vel
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("joystick_relay")
    try:
        SimpleJoystickRelay().run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting usb_joystick node")
