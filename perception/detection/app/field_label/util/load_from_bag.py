import numpy as np
import rospy
import tf2_py
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.messages.header import Header
from perception_tools.messages.camera_data import CameraData
from perception_tools.messages.image import Image
from perception_tools.messages.point_cloud import PointCloud
from rosbag.bag import Bag
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage


def load_from_bag(bag_file: str, cloud_topic: str, image_topic: str, info_topic: str) -> tuple[CameraData, Transform3D]:
    tf_buffer = tf2_py.BufferCore(cache_time=rospy.Duration.from_sec(1000000))  # type: ignore

    def update_buffer(msg: TFMessage) -> None:
        for msg_tf in msg.transforms:  # type: ignore
            tf_buffer.set_transform_static(msg_tf, "default_authority")

    point_cloud: PointCloud | None = None
    color_image: Image | None = None
    camera_info: CameraInfo | None = None
    with Bag(bag_file, "r") as bag:
        start_time = rospy.Time(bag.get_start_time())
        for topic, msg, timestamp in bag.read_messages(  # type: ignore
            topics=[cloud_topic, image_topic, info_topic, "/tf", "/tf_static"]
        ):
            point_cloud_found = point_cloud is not None
            color_image_found = color_image is not None
            camera_info_found = camera_info is not None
            if timestamp - start_time > rospy.Duration.from_sec(100.0):
                raise RuntimeError(
                    "Not all topics found in bag\n"
                    f"Cloud found: {point_cloud_found}\n"
                    f"Image found: {color_image_found}.\n"
                    f"Info found {camera_info_found}"
                )
            if topic == cloud_topic:
                point_cloud = PointCloud.from_msg(msg)
            elif topic == image_topic:
                color_image = Image.from_msg(msg)
            elif topic == info_topic:
                camera_info = msg
            elif topic == "/tf":
                update_buffer(msg)
            elif topic == "/tf_static":
                update_buffer(msg)

            if point_cloud_found and camera_info_found:
                break
    assert point_cloud is not None and camera_info is not None
    if color_image is None:
        color_image = Image(
            Header.from_msg(camera_info.header), np.zeros((camera_info.height, camera_info.width, 3), dtype=np.uint8)
        )
        print("No color image found in bag, using blank image")
    assert color_image is not None
    assert camera_info.header.frame_id == color_image.header.frame_id

    transform = tf_buffer.lookup_transform_core(point_cloud.header.frame_id, color_image.header.frame_id, rospy.Time(0))
    tf_pointcloud_from_camera = Transform3D.from_msg(transform.transform)

    return CameraData(
        color_image=color_image, point_cloud=point_cloud, camera_info=camera_info
    ), tf_pointcloud_from_camera
