from pathlib import Path
from typing import Any, Generator

import cv2
import numpy as np
from app.config.metrics_tool.metrics_tool_config import MetricsToolConfig
from app.metrics.list_cache_image_paths import list_cache_image_paths, list_image_timestamps
from app.metrics.load_interesting_video_frames import load_interesting_video_frames
from app.metrics.tracker.tracker import TrackerInterface
from bw_shared.configs.maps_config import MapConfig
from bw_shared.geometry.camera.camera_info_loader import read_calibration
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.xyz import XYZ
from bw_shared.messages.header import Header
from geometry_msgs.msg import Vector3
from image_geometry import PinholeCameraModel
from matplotlib import pyplot as plt
from perception_tools.messages.image import Image


def iterate_cached_frames(cache_dir: Path) -> Generator[Image, None, None]:
    for frame_num, image_path in enumerate(list_cache_image_paths(cache_dir)):
        print(f"Loading {image_path}")
        image = cv2.imread(str(image_path))
        header = Header(stamp=float(image_path.stem), frame_id="", seq=frame_num)
        yield Image(header=header, data=image)


def load_image_near_time(cache_dir: Path, time: float) -> Image:
    paths = list_cache_image_paths(cache_dir)
    closest_index = int(np.argmin([abs(float(p.stem) / 1000 - time) for p in paths]))
    closest_image_path = paths[closest_index]
    image = cv2.imread(str(closest_image_path))
    header = Header(stamp=float(closest_image_path.stem) / 1000, frame_id="", seq=closest_index)
    return Image(header=header, data=image)


class MetricsApp:
    def __init__(
        self,
        video_path: Path,
        config_path: Path,
        config: MetricsToolConfig,
        map_config: MapConfig,
        tracker: TrackerInterface,
    ) -> None:
        self.video_path = video_path
        self.config = config
        self.map_config = map_config
        self.tracker = tracker

        self.cached_image_dir = Path(self.video_path.parent / (self.video_path.stem + "_images"))

        intrinsics_path = Path(self.config.camera.intrinsics)
        if not intrinsics_path.is_absolute():
            intrinsics_path = config_path.parent / intrinsics_path
        self.camera_info = read_calibration(str(intrinsics_path))
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)

        self.tf_map_from_camera = Transform3D.from_position_and_rpy(
            Vector3(*self.config.camera.position.to_tuple()), self.config.camera.rotation.to_rpy()
        )

        self.current_object_id = 0
        self.current_frame = Image(data=np.zeros((300, 300, 3), dtype=np.uint8))

        self.window_name = "metrics"
        cv2.namedWindow(self.window_name)

    def _load_interesting_images(self) -> None:
        if self.cached_image_dir.exists():
            return
        print(f"Caching video frames to {self.cached_image_dir}")
        self.cached_image_dir.mkdir()
        for image in load_interesting_video_frames(
            self.video_path,
            self.config.video_filter,
            self.config.field,
            self.map_config.size,
            self.tf_map_from_camera,
            self.camera_info,
        ):
            stamp_msec = int(image.header.stamp * 1000)
            cv2.imwrite(str(self.cached_image_dir / f"{stamp_msec:010d}.jpg"), image.data)
            cv2.imshow(self.window_name, image.data)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
        paths = list_cache_image_paths(self.cached_image_dir)
        num_to_delete = max(0, len(paths) - self.config.video_filter.max_frames)
        keep_indices = np.round(np.linspace(0, len(paths) - 1, self.config.video_filter.max_frames)).astype(int)
        delete_indices = np.delete(np.arange(len(paths)), keep_indices)
        selected_paths: list[Path] = [paths[i] for i in delete_indices]
        for sampled_path in selected_paths:
            sampled_path.unlink()
        print(f"Finished caching {len(paths) - num_to_delete} video frames to {self.cached_image_dir}")

    def _on_mouse(self, event: int, x: int, y: int, flags: int, param: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            if flags & cv2.EVENT_FLAG_SHIFTKEY:
                print(f"Rejecting point at {x}, {y}")
                self.tracker.add_reject_point(self.current_frame.header.seq, self.current_object_id, (x, y))
            else:
                print(f"Tracking point at {x}, {y}")
                self.tracker.add_track_point(self.current_frame.header.seq, self.current_object_id, (x, y))

        else:
            return

    def _update_tracking(self) -> None:
        self.tracker.compute()

    def _draw_tracking(self, image: Image) -> np.ndarray:
        frame = image.data
        frame_num = image.header.seq
        if tracking_data := self.tracker.get_tracking(frame_num):
            draw_frame = frame.copy()
            self._draw_tracked_points(frame_num, draw_frame)
            cmap = plt.get_cmap("tab10")
            for obj_id, contours in tracking_data.contours.items():
                color = tuple((np.array([*cmap(obj_id)[:3], 0.6]) * 255).astype(np.uint8).tolist())
                for contour in contours:
                    cv2.drawContours(draw_frame, [contour], -1, color, 2)
        else:
            draw_frame = frame
        return draw_frame

    def _get_world_points(self, image: Image) -> dict[int, XYZ]:
        frame_num = image.header.seq
        if tracking_data := self.tracker.get_tracking(frame_num):
            world_points = {}
            for obj_id, contours in tracking_data.contours.items():
                world_points[obj_id] = self._contour_to_world_point(contours[0])
            return world_points
        else:
            return {}

    def _contour_to_world_point(self, contour: np.ndarray) -> XYZ:
        centroid = np.mean(contour, axis=0).astype(np.int32)
        ray = self.camera_model.projectPixelTo3dRay(centroid)

    def _draw_tracked_points(self, frame_num: int, frame: np.ndarray) -> None:
        for points in self.tracker.get_tracked_points().get((frame_num, 1), {}).values():
            for point in points:
                cv2.circle(frame, point, 3, (0, 255, 0), -1)
        for points in self.tracker.get_tracked_points().get((frame_num, 0), {}).values():
            for point in points:
                cv2.circle(frame, point, 3, (0, 0, 255), -1)

    def _track_selection_loop(self) -> None:
        self.current_frame = load_image_near_time(self.cached_image_dir, 0.0)
        cv2.imshow(self.window_name, self.current_frame.data)
        cv2.setMouseCallback(self.window_name, self._on_mouse)
        while True:
            cv2.imshow(self.window_name, self._draw_tracking(self.current_frame))
            key = chr(cv2.waitKey(50) & 0xFF)
            if key == "q":
                break
            elif key == "n":
                input_str = input("Enter time to jump to: ")
                try:
                    input_time = float(input_str)
                except ValueError:
                    print("Invalid input")
                    continue
                self.current_frame = load_image_near_time(self.cached_image_dir, input_time)
                print(f"Jumping to index {self.current_frame.header.seq}")
            elif key == "t":
                print("Updating tracking")
                self._update_tracking()
            elif key == "r":
                print("Resetting tracker")
                self.tracker.reset()
            elif key == "s":
                print("Saving tracking data as video")
                self._save_video()
            elif key.isdigit():
                self.current_object_id = (int(key) - 1) % 10
                print(f"Selected object ID {self.current_object_id}")

    def _save_video(self) -> None:
        height, width = self.current_frame.data.shape[0:2]
        out_video_path = self.cached_image_dir.parent / f"{self.cached_image_dir.name}_tracked.avi"
        timestamps = list_image_timestamps(self.cached_image_dir)
        average_frame_rate = float(1 / np.mean(np.diff(timestamps)))
        out_video = cv2.VideoWriter(
            str(out_video_path), cv2.VideoWriter_fourcc(*"XVID"), average_frame_rate, (width, height)
        )
        for image in iterate_cached_frames(self.cached_image_dir):
            out_video.write(self._draw_tracking(image))
        out_video.release()

    def run(self) -> None:
        self._load_interesting_images()

        self.tracker.initialize(self.cached_image_dir)

        self._track_selection_loop()
        cv2.destroyAllWindows()
