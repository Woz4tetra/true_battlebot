import logging
import shutil
from pathlib import Path

import cv2
import tqdm

from auto_label.backend.video_frame_database import VideoFrameDatabase

ALLOWED_VIDEO_EXTENSIONS = {".mp4", ".avi", ".mov", ".mkv"}


class VideoSourceCollection:
    def __init__(self, root_path: Path) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.images_path = root_path / "images"
        self.images_path.mkdir(parents=True, exist_ok=True)
        self.image_dirs: list[Path] = []
        self._selected_video: VideoFrameDatabase | None = None
        self.max_width = 1920
        self.max_height = 1080
        self._load_videos()

    def clear_selected(self) -> None:
        if self._selected_video is not None:
            self._selected_video.clear()

    def _load_videos(self) -> None:
        for video_path in self.images_path.iterdir():
            self.image_dirs.append(video_path)

    def add_video(self, video_path: Path) -> None:
        if not video_path.exists():
            self.logger.error(f"Video file {video_path} does not exist.")
            return
        if video_path.suffix.lower() not in ALLOWED_VIDEO_EXTENSIONS:
            self.logger.error(f"Unsupported video format: {video_path.suffix}.")
            return
        images_path = self.images_path / video_path.name
        if images_path in self.image_dirs:
            self.logger.warning(f"{images_path} already exists in the database.")
            return
        if images_path == video_path:
            self.logger.warning(f"Images path {images_path} is the same as video path {video_path}.")
            return
        if images_path.exists():
            self.logger.warning(f"Images path {images_path} already exists. Removing it.")
            shutil.rmtree(images_path)
        self.logger.info(f"Creating images directory for {video_path} at {images_path}.")
        images_path.mkdir(parents=True, exist_ok=True)
        self._generate_images_from_video(video_path)
        self.image_dirs.append(images_path)

    def _generate_images_from_video(self, video_path: Path) -> None:
        if not video_path.exists():
            self.logger.error(f"Video file {video_path} does not exist.")
            return
        self.logger.debug(f"Generating images from video {video_path}.")
        capture = cv2.VideoCapture(str(video_path))
        frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        pbar = tqdm.tqdm(total=frame_count, desc=f"Processing {video_path.name}", unit="frame")
        for frame_num in range(frame_count):
            pbar.update(1)
            ret, frame = capture.read()
            if not ret:
                break
            image_path = self.images_path / video_path.name / f"{frame_num:06d}.jpg"
            frame = self._resize_image(frame)
            cv2.imwrite(str(image_path), frame)
        capture.release()
        pbar.close()
        self.logger.info(f"Images generated for video {video_path} at {self.images_path / video_path.name}.")

    def _resize_image(self, image: cv2.Mat) -> cv2.Mat:
        height, width = image.shape[:2]
        if width > self.max_width or height > self.max_height:
            aspect_ratio = width / height
            if aspect_ratio > 1:
                new_width = self.max_width
                new_height = int(self.max_width / aspect_ratio)
            else:
                new_height = self.max_height
                new_width = int(self.max_height * aspect_ratio)
            return cv2.resize(image, (new_width, new_height))
        return image

    def select_video(self, video_name: str) -> VideoFrameDatabase | None:
        if self._selected_video is not None and self._selected_video.images_path.name == video_name:
            self.logger.debug(f"Video {video_name} is already selected.")
            return self._selected_video
        for image_dir in self.image_dirs:
            if image_dir.stem == video_name:
                if self._selected_video is not None:
                    self._selected_video.close()
                self._selected_video = VideoFrameDatabase(image_dir)
                self.logger.info(f"Selected video: {video_name}.")
                return self._selected_video
        self.logger.warning(f"Video {video_name} not found in the database.")
        return None

    def list_videos(self) -> list[str]:
        return [video.stem for video in self.image_dirs]
