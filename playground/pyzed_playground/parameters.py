import pyzed.sl as sl

camera = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
init_params.coordinate_units = sl.UNIT.METER
init_params.camera_resolution = sl.RESOLUTION.HD1080
# init_params.set_from_serial_number(serial_number, sl.BUS_TYPE.USB)
camera.open()
camera_information = camera.get_camera_information()
intrinsics = camera_information.camera_configuration.calibration_parameters.left_cam
print(intrinsics)
breakpoint()
