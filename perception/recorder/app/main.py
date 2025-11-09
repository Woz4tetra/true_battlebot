import time

from recorder_cpp import DEPTH_MODE, RESOLUTION, InitParameters, ZEDCamera


def main() -> None:
    init_parameters = InitParameters()
    init_parameters.camera_resolution = RESOLUTION.HD1080
    init_parameters.depth_mode = DEPTH_MODE.NONE
    init_parameters.camera_fps = 30
    zed = ZEDCamera(init_parameters)
    if not zed.open():
        print("Failed to open camera:", zed.get_last_error())
        exit(1)

    zed.start_recording("zed_recording.svo")
    time.sleep(5)
    zed.stop_recording()

    zed.close()
    print("Camera closed.")
