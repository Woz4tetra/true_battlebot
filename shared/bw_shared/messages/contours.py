import cv2
import numpy as np
from bw_interfaces.msg import Contour, UVKeypoint


def contour_to_msg(contours: list[np.ndarray]) -> list[Contour]:
    contour_msgs = []
    for contour in contours:
        points = [UVKeypoint(x, y) for x, y in contour]
        area = cv2.contourArea(contour.astype(np.int32))
        contour_msg = Contour(points=points, area=area)
        contour_msgs.append(contour_msg)
    return contour_msgs
