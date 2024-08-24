import argparse
import json
import random
import time
from dataclasses import dataclass, field
from pathlib import Path
from threading import Lock

import cv2
import numpy as np
import rospy
from bw_interfaces.msg import (
    ConfigureSimulation,
    EstimatedObject,
    EstimatedObjectArray,
    SegmentationInstanceArray,
    SimulationConfig,
)
from bw_shared.enums.label import OPPONENT_GROUP, ModelLabel
from bw_shared.geometry.projection_math.look_rotation import look_rotation
from bw_shared.geometry.projection_math.project_object_to_uv import ProjectionError, project_object_to_uv
from bw_shared.geometry.transform3d import Transform3D
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, Vector3
from image_geometry import PinholeCameraModel
from perception_tools.inference.simulated_mask_to_contours import (
    make_simulated_segmentation_color_map,
    segmentation_array_to_contour_map,
    simulated_mask_to_contours,
)
from perception_tools.rosbridge.wait_for_ros_connection import wait_for_ros_connection
from perception_tools.training.yolo_keypoint_dataset import (
    YoloKeypointAnnotation,
    YoloKeypointDataset,
    YoloKeypointImage,
    YoloVisibility,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

BRIDGE = CvBridge()
ALL_LABELS = (
    ModelLabel.MINI_BOT,
    ModelLabel.MAIN_BOT,
    ModelLabel.ROBOT,
    ModelLabel.REFEREE,
)


@dataclass
class DataShapshot:
    model: PinholeCameraModel | None = None
    image: np.ndarray | None = None
    layer: np.ndarray | None = None
    robots: EstimatedObjectArray | None = None
    dataset: YoloKeypointDataset = field(default_factory=YoloKeypointDataset)
    color_to_model_label_map: dict[int, ModelLabel] = field(default_factory=dict)
    image_timestamp: float = 0.0
    lock: Lock = field(default_factory=Lock)


IMAGE_LAYER_MAX_DELAY = 0.001
IMAGE_GROUND_MAX_DELAY = 0.015
CACHE_SIZE = 5
LAYER_CACHE = {}
GROUND_TRUTH_CACHE = {}


def image_callback(data_snapshot: DataShapshot, msg: Image) -> None:
    with data_snapshot.lock:
        if data_snapshot.image is not None:
            return
        data_snapshot.image = BRIDGE.imgmsg_to_cv2(msg, "bgr8")
        data_snapshot.image_timestamp = msg.header.stamp.to_sec()


def layer_callback(data_snapshot: DataShapshot, msg: Image) -> None:
    with data_snapshot.lock:
        layer = BRIDGE.imgmsg_to_cv2(msg)
        LAYER_CACHE[msg.header.stamp.to_sec()] = layer
        selected_key = min(LAYER_CACHE.keys(), key=lambda k: abs(k - data_snapshot.image_timestamp))
        selected_msg = LAYER_CACHE[selected_key]
        if abs(selected_key - data_snapshot.image_timestamp) <= IMAGE_LAYER_MAX_DELAY:
            data_snapshot.layer = selected_msg
        while len(LAYER_CACHE) > CACHE_SIZE:
            del LAYER_CACHE[min(LAYER_CACHE.keys())]


def camera_info_callback(data_snapshot: DataShapshot, msg: CameraInfo) -> None:
    with data_snapshot.lock:
        if data_snapshot.model is not None:
            return
        print("Received camera info.")
        data_snapshot.model = PinholeCameraModel()
        data_snapshot.model.fromCameraInfo(msg)


def ground_truth_callback(data_snapshot: DataShapshot, msg: EstimatedObjectArray) -> None:
    with data_snapshot.lock:
        GROUND_TRUTH_CACHE[msg.robots[-1].header.stamp.to_sec()] = msg
        selected_key = min(GROUND_TRUTH_CACHE.keys(), key=lambda k: abs(k - data_snapshot.image_timestamp))
        selected_msg = GROUND_TRUTH_CACHE[selected_key]
        if abs(selected_key - data_snapshot.image_timestamp) <= IMAGE_GROUND_MAX_DELAY:
            data_snapshot.robots = selected_msg
        else:
            print(f"Ground truth and image timestamps are too far apart. {selected_key -data_snapshot.image_timestamp}")
            data_snapshot.robots = None
        while len(GROUND_TRUTH_CACHE) > CACHE_SIZE:
            del GROUND_TRUTH_CACHE[min(GROUND_TRUTH_CACHE.keys())]


def simulated_segmentation_label_callback(data_snapshot: DataShapshot, msg: SegmentationInstanceArray) -> None:
    with data_snapshot.lock:
        color_to_model_label_map, skipped_labels = make_simulated_segmentation_color_map(msg, ALL_LABELS)
        if color_to_model_label_map != data_snapshot.color_to_model_label_map:
            data_snapshot.color_to_model_label_map = color_to_model_label_map
            labels = [label.value for label in data_snapshot.color_to_model_label_map.values()]
            print(f"Received labels: {str(labels)[1:-1]}. Skipped: {skipped_labels}")


def make_annotation_from_robot(
    robot: EstimatedObject,
    model: PinholeCameraModel,
    contour_map: dict[ModelLabel, list[np.ndarray]],
    image_size: tuple[int, int],
) -> YoloKeypointAnnotation | None:
    width, height = image_size
    label = ModelLabel(robot.label)
    if label in OPPONENT_GROUP:
        label = ModelLabel.ROBOT
    if label not in contour_map:
        print(f"Missing contour for label: {label}")
        return None
    contours = contour_map[label]
    try:
        forward, backward = project_object_to_uv(robot, model)
    except ProjectionError as e:
        print(f"Projection error: {e}")
        return None

    rectangles = np.array([cv2.boundingRect(contour) for contour in contours])
    top_left = np.min(rectangles[:, :2], axis=0)
    max_dims = np.max(rectangles[:, 2:], axis=0)
    bx, by, bw, bh = np.concatenate([top_left, max_dims])

    bx /= width
    by /= height
    bw /= width
    bh /= height

    keypoints = [(point.x / width, point.y / height, YoloVisibility.LABELED_VISIBLE) for point in (forward, backward)]
    return YoloKeypointAnnotation.from_xywh(bx, by, bw, bh, class_index=ALL_LABELS.index(label), keypoints=keypoints)


def record_image_and_keypoints(output_dir: Path, data_snapshot: DataShapshot) -> None:
    with data_snapshot.lock:
        if data_snapshot.image is None:
            print("Missing image")
            return
        image = data_snapshot.image.copy()
        data_snapshot.image = None
        if data_snapshot.layer is None:
            print("Missing layer")
            return
        if data_snapshot.robots is None:
            print("Missing robots")
            return
        if data_snapshot.model is None:
            print("Missing camera model")
            return
        if not data_snapshot.color_to_model_label_map:
            print("Missing color map data.")
            return
        filename = f"{time.time():.9f}"
        if data_snapshot.robots.robots[0].header.stamp.to_sec() - data_snapshot.image_timestamp > IMAGE_LAYER_MAX_DELAY:
            print("Robot and image timestamps are too far apart.")
            return
        layer = data_snapshot.layer.copy()
        robots = data_snapshot.robots
        model = data_snapshot.model
        image_annotation = YoloKeypointImage(image_id=filename)

        height, width = image.shape[:2]
        image_size = (width, height)

        segmentations, exceptions = simulated_mask_to_contours(
            layer, data_snapshot.color_to_model_label_map, ALL_LABELS
        )
        contour_map = segmentation_array_to_contour_map(segmentations)
        for robot in robots.robots:
            annotation = make_annotation_from_robot(robot, model, contour_map, image_size)
            if annotation is None:
                print(f"Skipping annotation. Label: {robot.label}")
                continue
            print(f"Adding annotation for label: {robot.label}")

            image_annotation.labels.append(annotation)

        image_path = str(output_dir / f"{filename}.jpg")
        print(f"Saving image to {image_path}")
        cv2.imwrite(image_path, image)

        with open(output_dir / f"{filename}.txt", "w") as file:
            file.write(image_annotation.to_txt())


def compute_camera_pose(distance: float, azimuth_angle: float, elevation_angle: float) -> Transform3D:
    position_array = np.array(
        [
            distance * np.cos(elevation_angle) * np.cos(azimuth_angle),
            -1 * distance * np.cos(elevation_angle) * np.sin(azimuth_angle),
            distance * np.sin(elevation_angle),
        ]
    )

    rotation = look_rotation(-1 * position_array)

    return Transform3D.from_position_and_quaternion(Vector3(*position_array), rotation)


def get_random_camera_pose() -> Transform3D:
    angle_inset = 0.2
    distance_range = (2.2, 2.5)
    azimuth_angle_range = (-np.pi / 4 + angle_inset, np.pi / 4 - angle_inset)
    elevation_angle_range = (0.4, 0.5)
    flip_azimuth = random.uniform(0.0, 1.0) > 0.5

    distance = random.uniform(*distance_range)
    azimuth_angle = random.uniform(*azimuth_angle_range)
    elevation_angle = random.uniform(*elevation_angle_range)

    if flip_azimuth:
        azimuth_angle = azimuth_angle + np.pi

    pose = compute_camera_pose(distance, azimuth_angle, elevation_angle)
    rotation = pose.quaternion_np
    randomized_rotation = np.random.normal(rotation, 0.01)
    camera_pose = Transform3D.from_position_and_quaternion(pose.position, Quaternion(*randomized_rotation))
    return camera_pose


def get_random_camera_objective() -> dict:
    camera_pose = get_random_camera_pose()

    rpy_deg = np.degrees(camera_pose.rpy)

    return {
        "type": "idle",
        "init": {
            "type": "flu",
            "x": camera_pose.position.x,
            "y": camera_pose.position.y,
            "z": camera_pose.position.z,
            "roll": rpy_deg[0],
            "pitch": rpy_deg[1],
            "yaw": rpy_deg[2],
        },
        "sequence": [],
    }


def generate_spawn_grid() -> list:
    x = np.linspace(-0.8, 0.8, 10)
    y = np.linspace(-0.8, 0.8, 10)
    grid = np.meshgrid(x, y)
    return np.array(grid).reshape(2, -1).T.tolist()


def select_from_grid(spawn_grid: list) -> tuple:
    return spawn_grid.pop(random.randint(0, len(spawn_grid) - 1))


def generate_angle() -> float:
    return random.uniform(0, 360)


def generate_target_objective(spawn_grid: list, target_name: str) -> dict:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    return {
        "type": "target",
        "init": {"type": "relative", "x": x, "y": y, "yaw": angle},
        "sequence": [{"target_name": target_name}],
    }


def generate_random_sequence(spawn_grid: list, duration: float) -> dict:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    timestamps = np.arange(0, duration, 0.5)
    sequence = []
    for timestamp in timestamps:
        angle = generate_angle()
        x = random.uniform(-0.8, 0.8)
        y = random.uniform(-0.8, 0.8)
        sequence.append(
            {
                "timestamp": timestamp,
                "x": x,
                "y": y,
                "yaw": angle,
                "vx": 10.0,
                "vyaw": 720.0,
            }
        )
    return {
        "type": "follow",
        "init": {
            "type": "relative",
            "x": x,
            "y": y,
            "yaw": angle,
        },
        "sequence": sequence,
    }


def generate_idle(spawn_grid: list) -> dict:
    angle = generate_angle()
    x, y = select_from_grid(spawn_grid)
    return {
        "type": "idle",
        "init": {"type": "relative", "x": x, "y": y, "yaw": angle},
        "sequence": [],
    }


def generate_scenario(num_bots: int) -> dict:
    cage_x = random.uniform(2.2, 2.4)
    cage_y = cage_x + random.uniform(-0.02, 0.02)
    spawn_referee = random.uniform(0, 1) > 0.5
    mini_bot_objective = "randomized_start_target_opponent" if num_bots >= 3 else "mini_bot_randomized_sequence"
    mini_bot = {
        "name": "mini_bot",
        "model": "MR STABS MK2",
        "objective": mini_bot_objective,
    }
    main_bot = {
        "name": "main_bot",
        "model": "MRS BUFF MK2",
        "objective": "main_bot_randomized_sequence",
    }
    opponent_1 = {
        "name": "opponent_1",
        "model": "MRS BUFF B-03 Bizarro",
        "objective": "randomized_start_target_main",
    }
    robot_actors = [mini_bot, main_bot, opponent_1]
    robot_actors = robot_actors[:num_bots]
    actors = [
        {"name": "tracking_camera", "model": "ZED 2i", "objective": "randomized_camera"},
    ] + robot_actors
    if spawn_referee:
        actors.append({"name": "referee", "model": "Referee", "objective": "randomized_idle"})
    return {
        "cage": {"dims": {"x": cage_x, "y": cage_y}},
        "actors": actors,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output_dir", type=str, default="output")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)

    if not output_dir.exists():
        output_dir.mkdir(parents=True)

    data_snapshot = DataShapshot()

    uri = wait_for_ros_connection()
    print(f"Connected to ROS master at {uri}")
    rospy.init_node("generate_images_from_sim")

    camera_ns = "/camera_0/"
    configure_simulation_pub = rospy.Publisher("/simulation/add_configuration", ConfigureSimulation, queue_size=1)
    select_scenario_pub = rospy.Publisher("/simulation/scenario_selection", String, queue_size=1)
    rospy.Subscriber(
        camera_ns + "ground_truth/robots",
        EstimatedObjectArray,
        lambda msg: ground_truth_callback(data_snapshot, msg),
        queue_size=1,
    )
    rospy.Subscriber(
        camera_ns + "rgb/camera_info",
        CameraInfo,
        lambda msg: camera_info_callback(data_snapshot, msg),
        queue_size=1,
    )
    rospy.Subscriber(
        camera_ns + "rgb/image_raw",
        Image,
        lambda msg: image_callback(data_snapshot, msg),
        queue_size=1,
    )
    rospy.Subscriber(
        camera_ns + "layer/image_raw",
        Image,
        lambda msg: layer_callback(data_snapshot, msg),
        queue_size=1,
    )
    rospy.Subscriber(
        camera_ns + "simulated_segmentation",
        SegmentationInstanceArray,
        lambda msg: simulated_segmentation_label_callback(data_snapshot, msg),
        queue_size=1,
    )

    rospy.sleep(2.0)  # wait for publishers to connect

    scenario_name = "image_synthesis"

    while not rospy.is_shutdown():
        # select number of robots and duration. Set camera pose.
        num_bots = random.randint(1, 3)
        duration = random.uniform(2, 5)

        # generate spawn grid
        spawn_grid = generate_spawn_grid()

        # generate camera objective
        camera_objective = SimulationConfig(
            name="randomized_camera",
            json_data=json.dumps(get_random_camera_objective()),
        )

        # generate random sequence for mini_bot
        mini_bot_random_sequence = SimulationConfig(
            name="mini_bot_randomized_sequence",
            json_data=json.dumps(generate_random_sequence(spawn_grid, duration)),
        )

        # generate target sequence for mini_bot
        mini_bot_target_sequence = SimulationConfig(
            name="randomized_start_target_opponent",
            json_data=json.dumps(generate_target_objective(spawn_grid, "opponent_1")),
        )

        # generate random sequence for main_bot
        main_bot_random_sequence = SimulationConfig(
            name="main_bot_randomized_sequence",
            json_data=json.dumps(generate_random_sequence(spawn_grid, duration)),
        )

        # generate target sequence for opponent_1
        opponent_1_target_sequence = SimulationConfig(
            name="randomized_start_target_main",
            json_data=json.dumps(generate_target_objective(spawn_grid, "main_bot")),
        )

        # generate idle sequence for referee
        referee_idle_sequence = SimulationConfig(
            name="randomized_idle",
            json_data=json.dumps(generate_idle(spawn_grid)),
        )

        # create scenario
        scenario = generate_scenario(num_bots)
        simulation_config = ConfigureSimulation(
            scenario=SimulationConfig(
                name=scenario_name,
                json_data=json.dumps(scenario),
            ),
            objectives=[
                camera_objective,
                mini_bot_random_sequence,
                mini_bot_target_sequence,
                main_bot_random_sequence,
                opponent_1_target_sequence,
                referee_idle_sequence,
            ],
        )

        # publish scenario and objectives. Start scenario.
        print(f"Starting scenario with {num_bots} robots for {duration} seconds.")
        configure_simulation_pub.publish(simulation_config)
        rospy.sleep(0.2)  # wait config to process
        select_scenario_pub.publish(scenario_name)

        # wait for scenario to finish. Repeat.
        start_time = time.monotonic()
        while time.monotonic() - start_time < duration:
            if rospy.is_shutdown():
                break
            record_image_and_keypoints(output_dir, data_snapshot)
            rospy.sleep(0.1)


if __name__ == "__main__":
    main()
