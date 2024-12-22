from typing import Optional

import cv2
import numpy as np
from bw_shared.camera_calibration.bundles.bundle_detector import compute_pose_ransac
from bw_shared.camera_calibration.detector.detector import Detector
from bw_shared.geometry.transform3d import Transform3D
from sensor_msgs.msg import CameraInfo


def make_ransac_params() -> cv2.UsacParams:
    microsac_params = cv2.UsacParams()
    microsac_params.threshold = 5.0
    microsac_params.confidence = 0.99999
    microsac_params.score = cv2.SCORE_METHOD_MSAC
    microsac_params.maxIterations = 10_000
    microsac_params.loIterations = 100
    microsac_params.loMethod = cv2.LOCAL_OPTIM_GC

    microsac_params.final_polisher = cv2.LSQ_POLISHER
    microsac_params.final_polisher_iterations = 10_000

    return microsac_params


def compute_board_pose(image: np.ndarray, camera_info: CameraInfo, detector: Detector) -> Optional[Transform3D]:
    detection_results = detector.detect(image)

    if detection_results is None:
        print("No detections found")
        return None
    object_points = detection_results.object_points
    image_points = detection_results.image_points
    return compute_pose_ransac(camera_info, make_ransac_params(), image_points, object_points)
