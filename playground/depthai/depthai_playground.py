import time

import cv2
import depthai as dai
import numpy as np
from bundle_detector import ArucoDetector, BundleConfig, BundleDetector, TagConfig
from draw_helpers import draw_bundle
from image_rectifier import ImageRectifier
from sensor_msgs.msg import CameraInfo


def main() -> None:
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define source and output
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    xout_video = pipeline.create(dai.node.XLinkOut)

    xout_video.setStreamName("video")

    width = 1920
    height = 1080

    # Properties
    cam_key = dai.CameraBoardSocket.CAM_A
    cam_rgb.setBoardSocket(cam_key)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setFps(30)

    xout_video.input.setBlocking(False)
    xout_video.input.setQueueSize(1)

    # Linking
    cam_rgb.video.link(xout_video.input)

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

        calib_data = device.readCalibration()
        instrinsics, info_width, info_height = calib_data.getDefaultIntrinsics(cam_key)
        if width is None:
            width = info_width
        if height is None:
            height = info_height

        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera_link"
        camera_info.width = info_width
        camera_info.height = info_height
        instrinsics = np.array(instrinsics)
        projection = np.zeros((3, 4))
        camera_info.K = instrinsics.reshape(9).tolist()
        camera_info.D = np.array(calib_data.getDistortionCoefficients(cam_key)).tolist()
        projection[:3, :3] = instrinsics[:3, :3]
        camera_info.P = projection.reshape(12).tolist()

        rectifier = ImageRectifier(camera_info, width, height)
        bundle_config = BundleConfig("bundle", [TagConfig(41, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)])

        microsac_params = cv2.UsacParams()
        microsac_params.threshold = 5.0
        microsac_params.confidence = 0.99999
        microsac_params.score = cv2.SCORE_METHOD_MSAC
        microsac_params.maxIterations = 10_000
        microsac_params.loIterations = 100
        microsac_params.loMethod = cv2.LOCAL_OPTIM_GC

        microsac_params.final_polisher = cv2.LSQ_POLISHER
        microsac_params.final_polisher_iterations = 10_000

        detector = ArucoDetector()
        rectified_info = rectifier.get_rectified_info()
        bundle_detector = BundleDetector(bundle_config, detector, microsac_params, rectified_info)

        time_samples = []
        while True:
            video_in = video.get()
            frame = video_in.getCvFrame()
            timestamp = video_in.getTimestamp().total_seconds()
            # timestamp = time.perf_counter()
            time_samples.append(timestamp)
            rectified = rectifier.rectify(frame)
            gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
            result = bundle_detector.detect(gray)
            draw_image = draw_bundle(rectified_info, rectified, result)

            cv2.imshow("video", draw_image)

            if cv2.waitKey(1) == ord("q"):
                break
            if len(time_samples) > 100:
                average_delta = np.mean(np.diff(time_samples))
                print(f"Average FPS: {1/average_delta}")
                time_samples = []


if __name__ == "__main__":
    main()
