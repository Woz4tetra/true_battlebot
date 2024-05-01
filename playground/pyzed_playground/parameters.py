import logging
import time

import numpy as np
import pyzed.sl as sl
from perception_tools.messages.camera.camera_info import CameraInfo


def main(camera: sl.Camera) -> None:
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
    init_params.coordinate_units = sl.UNIT.METER
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    # init_params.set_from_serial_number(serial_number, sl.BUS_TYPE.USB)
    runtime_parameters = sl.RuntimeParameters()

    camera.open(init_params)
    camera_information = camera.get_camera_information()
    intrinsics = camera_information.camera_configuration.calibration_parameters.left_cam
    resolution = intrinsics.image_size

    raw_intrinsics = [intrinsics.fx, 0, intrinsics.cx, 0, intrinsics.fy, intrinsics.cy, 0, 0, 1]

    height = resolution.height
    width = resolution.width

    camera_info = CameraInfo(
        height=height,
        width=width,
        distortion_model="plumb_bob",
        D=intrinsics.disto,
        K=raw_intrinsics,
    )
    print(camera_info)

    time_deltas = []
    for _ in range(20):
        t0 = time.perf_counter()
        status = camera.grab(runtime_parameters)
        t1 = time.perf_counter()
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"ZED Camera failed to grab frame: {status.name} ({status.value}): {str(status)}")
            return
        time_deltas.append(t1 - t0)
    print(f"Mean time: {np.mean(time_deltas)}")

    time_image = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds() * 1e-9
    time_current = camera.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_nanoseconds() * 1e-9

    print(time_current - time_image)
    print(time.time() - time_image)

    point_cloud = sl.Mat()
    camera.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

    raw_cloud_data = np.array(point_cloud.get_data(), dtype=np.float32)
    points = raw_cloud_data[..., 0:3]
    colors = raw_cloud_data[..., 3].view(np.uint32)

    cloud_dtype = np.dtype(
        {
            "names": ["x", "y", "z", "rgb"],
            "formats": [np.float32, np.float32, np.float32, np.uint32],
            "offsets": [0, 4, 8, 12],
        }
    )
    cloud_data = np.empty((height, width), dtype=cloud_dtype)
    cloud_data["x"] = points[..., 0]
    cloud_data["y"] = points[..., 1]
    cloud_data["z"] = points[..., 2]
    cloud_data["rgb"] = colors
    breakpoint()


try:
    camera = sl.Camera()

    main(camera)
finally:
    camera.close()
