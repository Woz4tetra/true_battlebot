from typing import cast

import numpy as np
import open3d as o3d
from app.field_label.command_line_args import BagCommandLineArgs, CommandLineArgs, TopicCommandLineArgs
from open3d.visualization.draw import draw
from perception_tools.messages.point_cloud import from_uint32_color, rospointcloud_to_array
from rosbag.bag import Bag
from sensor_msgs.msg import PointCloud2 as RosPointCloud


def ros_to_open3d(cloud_msg: RosPointCloud, max_value: float = 10.0) -> o3d.geometry.PointCloud:
    cloud_array = rospointcloud_to_array(cloud_msg)

    x = cloud_array["x"].view(np.float32)
    y = cloud_array["y"].view(np.float32)
    z = cloud_array["z"].view(np.float32)
    points = np.stack((x, y, z), axis=-1)
    delete_points = np.isnan(points) | ((points > max_value) | (points < -max_value))
    points = points[~np.any(delete_points, axis=1)]

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)

    compatible_color_fields = ["rgba", "rgb", "bgra", "rgb"]
    for color_field in compatible_color_fields:
        if color_field in cloud_array.dtype.names:  # type: ignore
            colors = from_uint32_color(cloud_array[color_field].view(np.uint32), color_field)
            colors = colors[~np.any(delete_points, axis=1)]
            cloud.colors = o3d.utility.Vector3dVector(colors)
            break

    return cloud


def load_first_cloud_from_bag(bag_file: str, topic: str) -> o3d.geometry.PointCloud:
    with Bag(bag_file, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=topic):
            return ros_to_open3d(msg)

    raise RuntimeError("No PointCloud2 messages found in bag file")


class FieldLabelApp:
    def __init__(self, args: CommandLineArgs) -> None:
        self.args = args

    def run_bag(self, args: BagCommandLineArgs) -> None:
        point_cloud = load_first_cloud_from_bag(args.bag_file, args.cloud_topic)
        print(point_cloud)

        # Visualize the point cloud
        draw([point_cloud])

    def run_topic(self, args: TopicCommandLineArgs) -> None:
        pass

    def run(self) -> None:
        match self.args.command:
            case "bag":
                self.run_bag(cast(BagCommandLineArgs, self.args))
            case "topic":
                self.run_topic(cast(TopicCommandLineArgs, self.args))
            case _:
                raise RuntimeError(f"Unknown command: {self.args.command}")
