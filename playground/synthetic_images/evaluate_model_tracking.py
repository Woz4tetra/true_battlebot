from dataclasses import dataclass

import rospy
from bw_interfaces.msg import ConfigureSimulation, EstimatedObjectArray, LabelMap
from bw_shared.configs.model_to_system_labels_map import ModelToSystemLabelsMap
from bw_shared.enums.label import Label, ModelLabel
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.pose2d_stamped import Pose2DStamped
from bw_shared.messages.header import Header
from perception_tools.rosbridge.wait_for_ros_connection import wait_for_ros_connection
from std_msgs.msg import String


@dataclass
class AppData:
    label_map: ModelToSystemLabelsMap | None = None
    ground_truth_data: dict[Label, list[Pose2DStamped]] = {}
    estimation_data: dict[Label, list[Pose2DStamped]] = {}

    def append_ground_truth(self, robots: EstimatedObjectArray) -> None:
        if self.label_map is None:
            return
        for robot in robots.robots:
            model_label = ModelLabel(robot.label)
            pose = Pose2D.from_msg(robot.pose.pose)
            pose_stamped = Pose2DStamped(header=Header.from_msg(robot.header), pose=pose)
            self.ground_truth_data.setdefault(self.label_map.labels[model_label], []).append(pose_stamped)

    def append_estimation(self, robots: EstimatedObjectArray) -> None:
        if self.label_map is None:
            return
        for robot in robots.robots:
            label = Label(robot.label)
            pose = Pose2D.from_msg(robot.pose.pose)
            pose_stamped = Pose2DStamped(header=Header.from_msg(robot.header), pose=pose)
            self.estimation_data.setdefault(label, []).append(pose_stamped)


def label_map_callback(app: AppData, msg: LabelMap) -> None:
    app.label_map = ModelToSystemLabelsMap.from_msg(msg)


def ground_truth_callback(app: AppData, msg: EstimatedObjectArray) -> None:
    app.append_ground_truth(msg)


def estimation_callback(app: AppData, msg: EstimatedObjectArray) -> None:
    app.append_estimation(msg)


def main() -> None:
    uri = wait_for_ros_connection()
    print(f"Connected to ROS master at {uri}")
    rospy.init_node("evaluate_model_tracking")

    app = AppData()
    camera_ns = "/camera_0/"
    configure_simulation_pub = rospy.Publisher("/simulation/add_configuration", ConfigureSimulation, queue_size=1)
    select_scenario_pub = rospy.Publisher("/simulation/scenario_selection", String, queue_size=1)
    rospy.Subscriber(
        camera_ns + "ground_truth/robots",
        EstimatedObjectArray,
        lambda msg: ground_truth_callback(app, msg),
        queue_size=1,
    )
    rospy.Subscriber(
        camera_ns + "estimation/robots",
        EstimatedObjectArray,
        lambda msg: estimation_callback(app, msg),
        queue_size=1,
    )
    rospy.Subscriber(
        camera_ns + "robot/label_map",
        LabelMap,
        lambda msg: label_map_callback(app, msg),
        queue_size=1,
    )