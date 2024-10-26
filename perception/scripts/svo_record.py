import argparse
import time

import cv2
import pyzed.sl as sl


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--save-directory", default="/data/svo", type=str, help="Directory to save images")
    args = parser.parse_args()

    save_directory = args.save_directory
    output_path = f"{save_directory}/{time.time()}.svo"

    init_params = sl.InitParameters()

    # Create a ZED camera object
    zed = sl.Camera()

    init_params.depth_mode = sl.DEPTH_MODE.NEURAL_PLUS
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 30
    init_params.coordinate_units = sl.UNIT.METER
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.enable_depth = False

    zed.open(init_params)

    # Enable recording with the filename specified in argument
    recording_params = sl.RecordingParameters()
    recording_params.compression_mode = sl.SVO_COMPRESSION_MODE.H264
    recording_params.video_filename = output_path
    err = zed.enable_recording(recording_params)
    print(err)

    color_image = sl.Mat()

    try:
        while True:
            # Each new frame is added to the SVO file
            zed.grab(runtime_parameters)

            zed.retrieve_image(color_image, sl.VIEW.LEFT)
            color_image_data = color_image.get_data()[..., 0:3]

            cv2.imshow("video", color_image_data)

            key = chr(cv2.waitKey(1) & 0xFF)
            if key == "q":
                break
            elif key == "p":
                timestamp = time.time()
                path = f"{save_directory}/image_{timestamp}.png"
                print(f"Saving photo to {path}")
                cv2.imwrite(path, color_image_data)

    finally:
        # Disable recording
        zed.disable_recording()


if __name__ == "__main__":
    main()
