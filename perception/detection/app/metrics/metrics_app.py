import csv
import pickle
from pathlib import Path
from typing import Any, Generator

import cv2
import numpy as np
from app.config.metrics_tool.metrics_tool_config import MetricsToolConfig
from app.metrics.list_cache_image_paths import list_cache_image_paths, list_image_timestamps
from app.metrics.tracker.tracker import TrackerInterface
from app.metrics.tracker.tracking_data import TrackingData
from app.metrics.video_prefilters import compute_static_background, find_blobs_with_background, iter_masked_images
from bw_shared.configs.maps_config import MapConfig
from bw_shared.geometry.camera.camera_info_loader import read_calibration
from bw_shared.geometry.projection_math.find_ray_plane_intersection import find_ray_plane_intersection
from bw_shared.geometry.transform3d import Transform3D
from bw_shared.geometry.transform_to_plane import transform_to_plane
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
        self.field_plane_center_in_camera, self.field_plane_normal_in_camera = transform_to_plane(
            self.tf_map_from_camera.inverse().tfmat
        )

        self.current_object_id = 0
        self.current_frame = Image(data=np.zeros((300, 300, 3), dtype=np.uint8))
        self.background = np.array([], dtype=np.uint8)

        self.window_name = "metrics"
        cv2.namedWindow(self.window_name)
        self.did_initialize = False
        self.color_cache: dict[int, tuple[int, ...]] = {}

    def _load_interesting_images(self) -> None:
        if self.cached_image_dir.exists():
            return
        print(f"Caching video frames to {self.cached_image_dir}")
        self.cached_image_dir.mkdir()
        for image in iter_masked_images(
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
            if not self.did_initialize:
                self.tracker.initialize(self.cached_image_dir)
                self.did_initialize = True
            if flags & cv2.EVENT_FLAG_SHIFTKEY:
                print(f"Rejecting point at {x}, {y}")
                self.tracker.add_reject_points(self.current_frame.header.seq, self.current_object_id, [(x, y)])
            else:
                print(f"Tracking point at {x}, {y}")
                self.tracker.add_track_points(self.current_frame.header.seq, self.current_object_id, [(x, y)])
        else:
            return

    def _auto_init_tracker(self, first_image: Image) -> None:
        points = find_blobs_with_background(self.background, first_image.data, self.config.video_filter)
        if not self.did_initialize:
            self.tracker.initialize(self.cached_image_dir)
            self.did_initialize = True
        for object_id, obj_points in points.items():
            self.tracker.add_track_points(first_image.header.seq, object_id, obj_points)

    def _update_tracking(self) -> None:
        self.tracker.compute()

    def _draw_tracking(self, image: Image) -> np.ndarray:
        frame = image.data
        frame_num = image.header.seq
        if tracking_data := self.tracker.get_tracking(frame_num):
            draw_frame = frame.copy()
            self._draw_tracked_points(frame_num, draw_frame)
            for obj_id, contours in tracking_data.contours.items():
                for contour in contours:
                    cv2.drawContours(draw_frame, [contour], -1, self._get_color(obj_id), 2)
            historic_data = self._get_historic_tracking_data(frame_num, look_behind_window=20)
            self._draw_historic_tracking_data(draw_frame, historic_data)
        else:
            draw_frame = frame
        return draw_frame

    def _get_color(self, obj_id: int) -> tuple[int, ...]:
        if obj_id not in self.color_cache:
            cmap = plt.get_cmap("tab10")
            color = tuple((np.array([*cmap(obj_id)[:3], 0.6]) * 255).astype(np.uint8).tolist())
            self.color_cache[obj_id] = color
        return self.color_cache[obj_id]

    def _get_historic_tracking_data(self, end_frame: int, look_behind_window: int) -> list[TrackingData]:
        start_frame = max(0, end_frame - look_behind_window)
        all_data = []
        for frame_num in range(start_frame, end_frame):
            if tracking_data := self.tracker.get_tracking(frame_num):
                all_data.append(tracking_data)
        return all_data

    def _draw_historic_tracking_data(self, frame: np.ndarray, all_data: list[TrackingData]) -> None:
        points: dict[int, list[np.ndarray]] = {}
        for tracking_data in all_data:
            for obj_id, contours in tracking_data.contours.items():
                for contour in contours:
                    centroid = np.mean(contour, axis=0).astype(np.int32)[0]
                    points.setdefault(obj_id, []).append(centroid)
        for obj_id, obj_points in points.items():
            color = self._get_color(obj_id)
            cv2.polylines(frame, [np.array(obj_points)], False, color, 2)

    def _get_map_points(self, frame_num: int) -> tuple[dict[int, XYZ], dict[int, XYZ]]:
        if tracking_data := self.tracker.get_tracking(frame_num):
            map_points = {}
            camera_pixels = {}
            for obj_id, contours in tracking_data.contours.items():
                if len(contours) == 0:
                    continue
                camera_point, centroid = self._contour_to_camera_point(contours[0])
                camera_pixels[obj_id] = XYZ(*centroid)
                map_points[obj_id] = self._camera_to_map_point(camera_point)
            return map_points, camera_pixels
        else:
            return {}, {}

    def _contour_to_camera_point(self, contour: np.ndarray) -> tuple[XYZ, np.ndarray]:
        centroid = np.mean(contour, axis=0).astype(np.int32)[0]
        ray = np.array(self.camera_model.projectPixelTo3dRay(centroid))
        intersection_in_camera = XYZ(
            *find_ray_plane_intersection(ray, self.field_plane_center_in_camera, self.field_plane_normal_in_camera)
        )
        return intersection_in_camera, centroid

    def _camera_to_map_point(self, intersection_in_camera: XYZ) -> XYZ:
        intersection_in_map = self.tf_map_from_camera.transform_point(intersection_in_camera)
        return intersection_in_map

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
        self._save_map_points()
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
                self._save_tracking_data()
                self._save_map_points()
            elif key == "r":
                print("Resetting tracker")
                self.tracker.reset()
            elif key == "s":
                self._save_video()
            elif key == "a":
                self._auto_init_tracker(self.current_frame)
            elif key.isdigit():
                self.current_object_id = (int(key) - 1) % 10
                print(f"Selected object ID {self.current_object_id}")

    def _save_video(self) -> None:
        print("Saving tracking data as video")
        height, width = self.current_frame.data.shape[0:2]
        out_video_path = self.cached_image_dir.parent / f"{self.cached_image_dir.name}_tracked.avi"
        timestamps = list_image_timestamps(self.cached_image_dir)
        average_frame_rate = float(1 / np.mean(np.diff(timestamps)))
        out_video = cv2.VideoWriter(
            str(out_video_path),
            cv2.VideoWriter_fourcc(*"XVID"),  # type: ignore
            average_frame_rate,
            (width, height),
        )
        for image in iterate_cached_frames(self.cached_image_dir):
            out_video.write(self._draw_tracking(image))
        out_video.release()

    def _save_tracking_data(self) -> None:
        out_pkl_path = self.cached_image_dir.parent / f"{self.cached_image_dir.name}_tracked.pkl"
        print(f"Saving tracking data to {out_pkl_path}")
        timestamps = list_image_timestamps(self.cached_image_dir)
        with open(out_pkl_path, "wb") as file:
            all_data = {}
            for frame_num in range(len(timestamps)):
                frame_tracking = self.tracker.get_tracking(frame_num)
                if frame_tracking is None:
                    continue
                all_data[frame_num] = frame_tracking
            pickle.dump(all_data, file)

    def _save_map_points(self) -> None:
        csv_path = self.cached_image_dir.parent / f"{self.cached_image_dir.name}_map_points.csv"
        print(f"Saving map points to {csv_path}")
        timestamps = list_image_timestamps(self.cached_image_dir)
        with open(csv_path, "w") as file:
            csv_writer = csv.DictWriter(
                file, fieldnames=["timestamp", "object_id", "map_x", "map_y", "camera_x", "camera_y"]
            )
            csv_writer.writeheader()
            for frame_num in range(len(timestamps)):
                map_points, camera_pixels = self._get_map_points(frame_num)
                for obj_id, map_point in map_points.items():
                    camera_px = camera_pixels[obj_id]
                    csv_writer.writerow(
                        {
                            "timestamp": timestamps[frame_num],
                            "object_id": obj_id,
                            "map_x": map_point.x,
                            "map_y": map_point.y,
                            "camera_x": camera_px[0],
                            "camera_y": camera_px[1],
                        }
                    )

    def _load_tracking_data(self) -> None:
        pkl_path = self.cached_image_dir.parent / f"{self.cached_image_dir.name}_tracked.pkl"
        if not pkl_path.exists():
            print(f"Tracking data file {pkl_path} does not exist")
            return
        with open(pkl_path, "rb") as file:
            all_data = pickle.load(file)
            for frame_num, frame_tracking in all_data.items():
                self.tracker.set_tracking(frame_num, frame_tracking)

    def _compute_tracker_initialization(self) -> np.ndarray:
        saved_background_path = self.cached_image_dir.parent / f"{self.cached_image_dir.name}_background.jpg"
        if saved_background_path.exists():
            background = cv2.imread(str(saved_background_path))
            print(f"Loaded background from {saved_background_path}")
        else:
            print("Computing static background")
            background = compute_static_background(
                iterate_cached_frames(self.cached_image_dir), self.config.video_filter
            ).data
            cv2.imwrite(str(saved_background_path), background)
            print(f"Saved background to {saved_background_path}")
        return background

    def run(self) -> None:
        self._load_interesting_images()
        self.background = self._compute_tracker_initialization()
        self._load_tracking_data()
        self._track_selection_loop()
        cv2.destroyAllWindows()
