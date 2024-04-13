import rospy
from bw_tools.transforms import lookup_pose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def main() -> None:
    rospy.init_node("optical_camera_to_map_pose_generator")
    publisher = rospy.Publisher("optical_camera_to_map_pose", PoseStamped, queue_size=1, latch=True)
    buffer = Buffer()
    listener = TransformListener(buffer)
    pose = lookup_pose(buffer, "camera_0_left_camera_optical_frame", "map", timeout=rospy.Duration(10))
    publisher.publish(pose)
    print("done")
    rospy.spin()


if __name__ == "__main__":
    main()
