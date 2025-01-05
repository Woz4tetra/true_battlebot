from pathlib import Path

import cv2
from app.config.metrics_tool_config.load_config import load_config
from app.metrics.command_line_args import CommandLineArgs
from app.metrics.load_interesting_video_frames import load_interesting_video_frames
from bw_shared.configs.shared_config import SharedConfig
from bw_shared.geometry.camera.camera_info_loader import read_calibration
from bw_shared.geometry.transform3d import Transform3D
from geometry_msgs.msg import Vector3


def run_app(args: CommandLineArgs) -> None:
    config_path = Path(args.config)
    video_path = Path(args.video_file)

    config = load_config(config_path)

    shared_config = SharedConfig.from_files()
    map_config = shared_config.get_map(config.field.type)

    intrinsics_path = Path(config.camera.intrinsics)
    if not intrinsics_path.is_absolute():
        intrinsics_path = config_path.parent / intrinsics_path
    camera_info = read_calibration(str(intrinsics_path))

    tf_map_from_camera = Transform3D.from_position_and_rpy(
        Vector3(*config.camera.position.to_tuple()), config.camera.rotation.to_rpy()
    )
    window_name = "frame"
    cv2.namedWindow(window_name)
    for image in load_interesting_video_frames(
        video_path, config.video_filter, config.field, map_config.size, tf_map_from_camera, camera_info
    ):
        cv2.imshow(window_name, image.data)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
