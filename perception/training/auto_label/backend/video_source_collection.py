import logging
from pathlib import Path

from auto_label.backend.video_frame_database import VideoFrameCache

ALLOWED_VIDEO_EXTENSIONS = {".mp4", ".avi", ".mov", ".mkv"}


class VideoSourceCollection:
    def __init__(self, root_path: Path) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.root_path = root_path
        self.videos: list[Path] = []
        self._selected_video: VideoFrameCache | None = None
        self._load_videos()

    def clear_selected(self) -> None:
        if self._selected_video is not None:
            self._selected_video.clear()

    def _load_videos(self) -> None:
        for video_path in self.root_path.iterdir():
            if video_path.suffix.lower() in ALLOWED_VIDEO_EXTENSIONS:
                self.videos.append(video_path)

    def add_video(self, video_path: Path) -> None:
        if not video_path.exists():
            self.logger.error(f"Video file {video_path} does not exist.")
            return
        if video_path.suffix.lower() not in ALLOWED_VIDEO_EXTENSIONS:
            self.logger.error(f"Unsupported video format: {video_path.suffix}.")
            return
        if video_path in self.videos:
            self.logger.warning(f"Video {video_path} already exists in the database.")
            return
        symlink_path = self.root_path / video_path.name
        if symlink_path == video_path:
            self.logger.warning(f"Symlink path {symlink_path} is the same as video path {video_path}.")
            return
        if symlink_path.exists():
            self.logger.warning(f"Symlink {symlink_path} already exists. Removing it.")
            symlink_path.unlink()
        self.logger.info(f"Creating symlink for {video_path} at {symlink_path}.")
        symlink_path.symlink_to(video_path)
        self.videos.append(video_path)

    def select_video(self, video_name: str) -> VideoFrameCache | None:
        if self._selected_video is not None and self._selected_video.video_path.name == video_name:
            self.logger.debug(f"Video {video_name} is already selected.")
            return self._selected_video
        for video in self.videos:
            if video.stem == video_name:
                if self._selected_video is not None:
                    self._selected_video.close()
                self._selected_video = VideoFrameCache(video)
                self.logger.info(f"Selected video: {video_name}.")
                return self._selected_video
        self.logger.warning(f"Video {video_name} not found in the database.")
        return None

    def list_videos(self) -> list[str]:
        return [video.stem for video in self.videos]
