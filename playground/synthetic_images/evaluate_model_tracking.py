import time
from dataclasses import dataclass, field

import rospy
from bw_interfaces.msg import EstimatedObject, EstimatedObjectArray, LabelMap
from bw_shared.configs.model_to_system_labels_map import ModelToSystemLabelsMap
from bw_shared.enums.label import Label, ModelLabel
from bw_shared.geometry.pose2d import Pose2D
from bw_shared.geometry.pose2d_stamped import Pose2DStamped
from bw_shared.messages.header import Header
from matplotlib import pyplot as plt
from perception_tools.rosbridge.wait_for_ros_connection import wait_for_ros_connection
from std_msgs.msg import Empty, String


@dataclass
class AppData:
    label_map: ModelToSystemLabelsMap | None = None
    estimated_field: EstimatedObject | None = None
    ground_truth_data: dict[Label, list[Pose2DStamped]] = field(default_factory=dict)
    estimation_data: dict[Label, list[Pose2DStamped]] = field(default_factory=dict)
    record_data: bool = False

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
    if not app.record_data:
        return
    app.append_ground_truth(msg)


def estimation_callback(app: AppData, msg: EstimatedObjectArray) -> None:
    if not app.record_data:
        return
    app.append_estimation(msg)


def field_callback(app: AppData, msg: EstimatedObject) -> None:
    app.estimated_field = msg


def main() -> None:
    uri = wait_for_ros_connection()
    print(f"Connected to ROS master at {uri}")
    rospy.init_node("evaluate_model_tracking")
    rospy.sleep(1.0)  # Wait for the node to initialize

    app = AppData()
    camera_ns = "/camera_0/"
    select_scenario_pub = rospy.Publisher("/simulation/scenario_selection", String, queue_size=1)
    request_field_pub = rospy.Publisher("/field_request", Empty, queue_size=1)
    rospy.Subscriber(
        "/filter/field",
        EstimatedObject,
        lambda msg: field_callback(app, msg),
    )
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

    print("Waiting for label map")
    while not rospy.is_shutdown():
        if app.label_map is not None:
            break
        time.sleep(0.01)

    print("Requesting field")
    request_time = rospy.Time.now()
    request_field_pub.publish(Empty())
    while not rospy.is_shutdown():
        if app.estimated_field is not None:
            break
        if rospy.Time.now() - request_time > rospy.Duration(5):
            print("Failed to receive field")
            return
        time.sleep(0.01)

    print("Selecting scenario")
    for _ in range(10):
        rospy.sleep(0.01)
        select_scenario_pub.publish("fight_1")
    rospy.sleep(0.2)  # Wait for the scenario to load

    print("Recording ground truth and estimation data")
    start_time = time.monotonic()
    duration = 5.0
    app.record_data = True
    while not rospy.is_shutdown():
        if time.monotonic() - start_time > duration:
            break
        time.sleep(0.01)

    fig, ax = plt.subplots()
    colors = {}
    for label, poses in app.ground_truth_data.items():
        x = [pose.pose.x for pose in poses]
        y = [pose.pose.y for pose in poses]
        line = ax.plot(x, y, label=label + " (GT)")[0]
        colors[label] = line.get_color()

    for label, poses in app.estimation_data.items():
        x = [pose.pose.x for pose in poses]
        y = [pose.pose.y for pose in poses]
        color = colors[label]
        ax.plot(x, y, label=label + " (Est)", color=color, marker=".", linestyle="None")

    ax.legend()
    plt.show()


if __name__ == "__main__":
    main()
