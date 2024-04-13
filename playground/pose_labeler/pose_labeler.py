import argparse
import copy
import json
import os
import string
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import cv2
import numpy as np
import rosbag
from bw_interfaces.msg import EstimatedObject
from bw_tools.projection_math.project_segmentation import line_plane_intersection
from bw_tools.structs.header import Header
from bw_tools.structs.pose2d import Pose2D
from bw_tools.structs.pose2d_stamped import Pose2DStamped
from bw_tools.structs.transform3d import Transform3D
from bw_tools.structs.xy import XY
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from image_geometry import PinholeCameraModel
from matplotlib import colormaps
from sensor_msgs.msg import CameraInfo
from tqdm import tqdm

ArrayNx3 = np.ndarray
ArrayNx2 = np.ndarray


class Projections:
    def __init__(
        self, camera_info: CameraInfo, optical_camera_to_map_pose: Pose, field_size: XY, warped_width: int
    ) -> None:
        self.field_size = field_size
        self.model = PinholeCameraModel()
        self.model.fromCameraInfo(camera_info)

        map_to_camera = Transform3D.from_pose_msg(optical_camera_to_map_pose)
        self.optical_camera_to_map_tf = map_to_camera.inverse().tfmat
        self.map_to_camera_tf = map_to_camera.tfmat

        normal_and_center = self.project_map_array_to_camera(np.array([[0, 0, 1], [0, 0, 0]]))
        self.plane_normal = normal_and_center[0] - normal_and_center[1]
        self.plane_normal = self.plane_normal / np.linalg.norm(self.plane_normal)
        self.plane_center = normal_and_center[1]

        field_corners_in_map = np.array([[1, 1, 0], [-1, 1, 0], [-1, -1, 0], [1, -1, 0]], dtype=np.float64)
        field_corners_in_map[:, 0] *= field_size.x / 2
        field_corners_in_map[:, 1] *= field_size.y / 2
        self.field_pixels = self.project_map_array_to_pixels(field_corners_in_map)

        warped_height = warped_width * field_size.y / field_size.x
        corner_pixels = np.array(
            [
                [0, warped_height],
                [warped_width, warped_height],
                [warped_width, 0],
                [0, 0],
            ],
            dtype=np.float32,
        )
        self.warped_width = int(warped_width)
        self.warped_height = int(warped_height)
        self.homography = np.array(cv2.getPerspectiveTransform(self.field_pixels, corner_pixels), dtype=np.float64)
        self.inv_homography = np.linalg.inv(self.homography)

    def project_map_point_to_pixel(self, point: Vector3) -> np.ndarray:
        point_in_map = np.array([point.x, point.y, point.z, 1], dtype=np.float64)
        point_in_camera = self.optical_camera_to_map_tf @ point_in_map
        return np.array(self.model.project3dToPixel(point_in_camera[:3]))

    def project_map_array_to_camera(self, points: ArrayNx3) -> ArrayNx3:
        points_in_map = np.hstack([points, np.ones((points.shape[0], 1))])
        points_in_camera = np.tensordot(points_in_map, self.optical_camera_to_map_tf, axes=(1, 1))
        points_in_camera = points_in_camera[:, 0:3]
        return points_in_camera

    def project_map_array_to_pixels(self, points: ArrayNx3) -> ArrayNx2:
        points_in_camera = self.project_map_array_to_camera(points)
        return np.array([self.model.project3dToPixel(point) for point in points_in_camera], dtype=np.float32)

    def project_pixel_to_unwarped(self, pixel: np.ndarray) -> np.ndarray:
        input_pixel = np.array([[pixel]], dtype=np.float32)
        return cv2.perspectiveTransform(input_pixel, self.inv_homography)[0][0].astype(np.int32)

    def project_pixel_to_map(self, pixel: np.ndarray) -> Optional[Vector3]:
        unwarped_pixel = self.project_pixel_to_unwarped(pixel)
        projected_ray = np.array(self.model.projectPixelTo3dRay((unwarped_pixel[0], unwarped_pixel[1])))
        output_pose_in_optical_camera = line_plane_intersection(
            projected_ray, projected_ray * 2, self.plane_center, self.plane_normal
        )
        if output_pose_in_optical_camera is None:
            return None
        output_pose_in_map = (self.map_to_camera_tf @ np.array([*output_pose_in_optical_camera, 1]))[:3]
        return Vector3(*output_pose_in_map)


class RobotLabel(Enum):
    MAIN_BOT = "main_bot"
    MINI_BOT = "mini_bot"
    OPPONENT_1 = "opponent_1"
    OPPONENT_2 = "opponent_2"
    REFEREE = "referee"


@dataclass
class ClickedState:
    start_pixel: Optional[np.ndarray] = None
    end_pixel: Optional[np.ndarray] = None
    pose: Optional[Pose2DStamped] = None


RecordedSample = dict[RobotLabel, ClickedState]


@dataclass
class RecordedPoses:
    poses: list[RecordedSample]

    def set_state(self, index: int, label: RobotLabel, state: ClickedState) -> None:
        self.poses[index][label] = state


@dataclass
class AppState:
    images: list[np.ndarray]
    timestamps: list[float]
    projections: Projections
    current_index: int
    window_name: str
    trackbar_name: str
    selected_label: RobotLabel
    label_colors: dict[RobotLabel, np.ndarray]
    recording: RecordedPoses
    prev_labeled_index: int = 0

    @property
    def current_frame(self) -> np.ndarray:
        return self.images[self.current_index]

    @property
    def current_color(self) -> tuple:
        return tuple(self.label_colors[self.selected_label].tolist())

    @property
    def current_timestamp(self) -> float:
        return self.timestamps[self.current_index]


def recorded_poses_to_json(recording: RecordedPoses, field_size: XY) -> dict[RobotLabel, dict]:
    timestamps = []
    for frame_data in recording.poses:
        for label, clicked in frame_data.items():
            if clicked.pose is not None:
                timestamps.append(clicked.pose.header.stamp)
    start_timestamp = min(timestamps)

    def make_path(poses: list[Pose2DStamped], size: XY) -> dict:
        max_x = size.x / 2
        max_y = size.y / 2
        init_pose = poses[0].pose
        return {
            "type": "follow",
            "timestamp": start_timestamp,
            "init": {
                "type": "relative",
                "x": init_pose.x / max_x,
                "y": init_pose.y / max_y,
                "theta": np.rad2deg(init_pose.theta),
            },
            "sequence": [
                {
                    "timestamp": pose.header.stamp - start_timestamp,
                    "x": pose.pose.x / max_x,
                    "y": pose.pose.y / max_y,
                    "theta": np.rad2deg(pose.pose.theta),
                }
                for pose in poses
            ],
        }

    paths = {label: [] for label in RobotLabel}
    for frame_data in recording.poses:
        for label, clicked in frame_data.items():
            if clicked.pose is not None:
                paths[label].append(clicked.pose)
    paths = {label: path for label, path in paths.items() if len(path) > 0}
    jsons = {label: make_path(path, field_size) for label, path in paths.items()}
    return jsons


def write_json_file(base_path: str, recording: RecordedPoses, field_size: XY) -> None:
    jsons = recorded_poses_to_json(recording, field_size)
    for label, data in jsons.items():
        path = base_path + f"_{label.value}.json"
        base_dir = os.path.dirname(path)
        os.makedirs(base_dir, exist_ok=True)
        print("Writing to", path)
        with open(path, "w") as f:
            json.dump(data, f, indent=4)


def get_shown_image(state: AppState) -> np.ndarray:
    image = state.current_frame
    projections = state.projections
    return draw_recording(
        cv2.warpPerspective(
            image,
            projections.homography,
            (projections.warped_width, projections.warped_height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        ),
        state.recording.poses[state.current_index],
        state.label_colors,
    )


def trackbar_callback(val: int, state: AppState) -> None:
    state.current_index = min(len(state.images) - 1, max(0, val))
    image = get_shown_image(state)
    cv2.imshow(state.window_name, image)


def draw_recorded_pose(
    image: np.ndarray, clicked: ClickedState, label: RobotLabel, color: tuple[int, int, int]
) -> np.ndarray:
    if clicked.start_pixel is not None and clicked.end_pixel is not None:
        image = cv2.arrowedLine(
            img=image,
            pt1=tuple(clicked.start_pixel),
            pt2=tuple(clicked.end_pixel),
            color=color,
            thickness=5,
        )
    if clicked.start_pixel is not None:
        text = label.value
        for thickness, color in ((10, (255, 255, 255)), (5, color)):
            image = cv2.putText(
                img=image,
                text=text,
                org=tuple(clicked.start_pixel),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1,
                color=color,
                thickness=thickness,
            )
    return image


def draw_recording(image: np.ndarray, sample: RecordedSample, label_colors: dict[RobotLabel, np.ndarray]) -> np.ndarray:
    for label, clicked in sample.items():
        if clicked.pose is not None:
            color = tuple(label_colors[label].tolist())
            image = draw_recorded_pose(image, clicked, label, color)
    return image


def mouse_callback(event: int, x: int, y: int, flags: int, param: tuple[AppState, ClickedState]) -> None:
    state, clicked = param
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked.start_pixel = np.array([x, y])
    elif event == cv2.EVENT_MOUSEMOVE:
        if clicked.start_pixel is not None:
            clicked.end_pixel = np.array([x, y])
            image = draw_recorded_pose(get_shown_image(state), clicked, state.selected_label, state.current_color)
            cv2.imshow(state.window_name, image)
    elif event == cv2.EVENT_LBUTTONUP:
        if clicked.start_pixel is None or clicked.end_pixel is None:
            print("Start or end point is not set")
            return
        start_pose = state.projections.project_pixel_to_map(clicked.start_pixel)
        end_pose = state.projections.project_pixel_to_map(clicked.end_pixel)
        if start_pose is None or end_pose is None:
            print("No intersection found")
            return
        start_2d = Pose2D(x=start_pose.x, y=start_pose.y, theta=0.0)
        end_2d = Pose2D(x=end_pose.x, y=end_pose.y, theta=0.0)
        heading = start_2d.heading(end_2d)
        pose_2d = Pose2D(x=start_pose.x, y=start_pose.y, theta=heading)
        print(pose_2d)
        clicked.pose = Pose2DStamped(Header(state.current_timestamp, "", state.current_index), pose_2d)
        state.recording.set_state(state.current_index, state.selected_label, copy.deepcopy(clicked))
        state.prev_labeled_index = state.current_index
        clicked.start_pixel = None
        cv2.imshow(state.window_name, get_shown_image(state))


def load_data(bag_path: str, temp_dir: str) -> tuple[EstimatedObject, CameraInfo, PoseStamped, list[float]]:
    bridge = CvBridge()
    bag = rosbag.Bag(bag_path)

    timestamps = []
    field = EstimatedObject()
    camera_info = CameraInfo()
    optical_camera_to_map_pose: Optional[PoseStamped] = None

    with tqdm(total=bag.get_message_count()) as pbar:
        for topic, msg, timestamp in bag.read_messages():
            if "filter/field" in topic and msg.state.header.frame_id and not field.state.header.frame_id:
                field = msg
            elif "/camera_0/rgb/camera_info" in topic and not camera_info.header.frame_id:
                camera_info = msg
            elif "optical_camera_to_map_pose" in topic and not optical_camera_to_map_pose:
                optical_camera_to_map_pose = msg
            elif "/camera_0/debug_image" in topic:
                image = bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imwrite(os.path.join(temp_dir, f"{msg.header.stamp.to_sec()}.png"), image)
                timestamps.append(msg.header.stamp.to_sec())
            pbar.update(1)
    bag.close()
    assert optical_camera_to_map_pose is not None
    return field, camera_info, optical_camera_to_map_pose, timestamps


def init_app(bag_path: str) -> AppState:
    field, camera_info, optical_camera_to_map_pose, timestamps = load_data(bag_path)

    warped_height = 1200
    projections = Projections(
        camera_info, optical_camera_to_map_pose.pose, XY(field.size.x, field.size.y), warped_height
    )
    center_pixel = np.array([projections.warped_width / 2, projections.warped_height / 2])
    print(projections.project_pixel_to_map(center_pixel))

    # fill colors based on matplotlib color map
    color_map = colormaps.get_cmap("Spectral")
    label_colors = {
        label: color * 255 for label, color in zip(RobotLabel, color_map(np.linspace(0, 1, len(RobotLabel))))
    }

    recording = RecordedPoses([{} for _ in range(len(images))])

    window_name = "Field"
    trackbar_name = "Frame"
    state = AppState(
        images, timestamps, projections, 0, window_name, trackbar_name, RobotLabel.MAIN_BOT, label_colors, recording
    )
    clicked = ClickedState()

    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar(trackbar_name, window_name, 0, len(images) - 1, lambda val: trackbar_callback(val, state))
    cv2.setMouseCallback(window_name, mouse_callback, (state, clicked))

    return state


def main() -> None:
    parser = argparse.ArgumentParser(description="Label poses in a rosbag")
    parser.add_argument("bagfile", type=str, help="Path to the rosbag file")
    args = parser.parse_args()
    bag_path = args.bagfile
    out_path = bag_path.replace(".bag", "_poses")

    label_mapping = {
        "1": RobotLabel.MAIN_BOT,
        "2": RobotLabel.MINI_BOT,
        "3": RobotLabel.OPPONENT_1,
        "4": RobotLabel.OPPONENT_2,
        "5": RobotLabel.REFEREE,
    }
    state = init_app(bag_path)
    field_size = state.projections.field_size

    def save_json():
        write_json_file(out_path, state.recording, XY(field_size.x, field_size.y))

    def draw():
        cv2.imshow(state.window_name, get_shown_image(state))

    cv2.imshow(state.window_name, get_shown_image(state))
    while True:
        keycode = cv2.waitKey()
        key = chr(keycode & 0xFF)
        if key == "q":
            save_json()
            break
        elif key == "a":
            state.current_index = max(0, state.current_index - 1)
            draw()
            cv2.setTrackbarPos(state.trackbar_name, state.window_name, state.current_index)
        elif key == "d":
            state.current_index = min(len(state.images) - 1, state.current_index + 1)
            draw()
            cv2.setTrackbarPos(state.trackbar_name, state.window_name, state.current_index)
        elif key == "l":
            # copy last frame's poses
            state.recording.poses[state.current_index] = copy.deepcopy(state.recording.poses[state.prev_labeled_index])
            for sample in state.recording.poses[state.current_index].values():
                if sample.pose is None:
                    continue
                sample.pose.header.stamp = state.current_timestamp
            draw()
        elif key == "c":
            # clear current selected label's pose
            state.recording.poses[state.current_index][state.selected_label] = ClickedState()
            draw()
        elif key in string.digits:
            state.selected_label = label_mapping.get(key, RobotLabel.MAIN_BOT)
            print("Selected", state.selected_label.value)
        elif key == "s":
            save_json()


if __name__ == "__main__":
    main()
