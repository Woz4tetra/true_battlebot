import sys
import time

import rospy
from bw_shared.geometry.transform3d import Transform3D
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud
from perception_tools.rosbridge.ros_poll_subscriber import RosPollSubscriber
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import PointCloud2 as RosPointCloud
from tf2_ros import Buffer


def load_from_topics(
    cloud_subscriber: RosPollSubscriber[RosPointCloud],
    image_subscriber: RosPollSubscriber[RosImage],
    info_subscriber: RosPollSubscriber[CameraInfo],
    tf_buffer: Buffer,
) -> tuple[CameraData, Transform3D]:
    point_cloud = None
    color_image = None
    camera_info = None
    while point_cloud is None or color_image is None or camera_info is None:
        if point_cloud is None and (cloud_msg := cloud_subscriber.receive()):
            point_cloud = PointCloud.from_msg(cloud_msg)
            print("Received point cloud")
        if color_image is None and (image_msg := image_subscriber.receive()):
            color_image = Image.from_msg(image_msg)
            print("Received color image")
        if camera_info is None and (info_msg := info_subscriber.receive()):
            camera_info = info_msg
            print("Received camera info")
        time.sleep(0.1)
        if rospy.is_shutdown():
            sys.exit(0)
    transform = tf_buffer.lookup_transform(point_cloud.header.frame_id, color_image.header.frame_id, rospy.Time(0))
    tf_pointcloud_from_camera = Transform3D.from_msg(transform.transform)
    return CameraData(
        color_image=color_image, point_cloud=point_cloud, camera_info=camera_info
    ), tf_pointcloud_from_camera
