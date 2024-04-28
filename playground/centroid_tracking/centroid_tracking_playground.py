from dataclasses import dataclass
from queue import Queue

import cv2
import numpy as np
import rospy
from centroid_tracker import CentroidTracker
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


@dataclass
class AppData:
    window_name: str
    tracker: CentroidTracker
    back_subtractor: cv2.BackgroundSubtractorMOG2
    min_area_percentage: float = 0.0005
    cv_bridge: CvBridge = CvBridge()
    out_queue: Queue = Queue()
    width: int = 1920
    learning_rate: float = -1


def image_callback(data: AppData, msg: Image) -> None:
    try:
        frame = data.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        print(e)
        return

    # tracker = data.tracker
    back_subtractor = data.back_subtractor
    out_queue = data.out_queue
    min_area_percentage = data.min_area_percentage

    image_area = frame.shape[0] * frame.shape[1]
    min_area = min_area_percentage * image_area

    fg_mask = back_subtractor.apply(frame, learningRate=data.learning_rate)
    # fg_mask = cv2.medianBlur(fg_mask, ksize=5)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel, iterations=3)

    contours, hierarchy = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    objects = []
    for contour in contours:
        if cv2.contourArea(contour) < min_area:
            continue
        rectangle = cv2.boxPoints(cv2.minAreaRect(contour))
        rectangle = rectangle.astype(np.int32)
        bbox = (
            np.min(rectangle[:, 0]),
            np.min(rectangle[:, 1]),
            np.max(rectangle[:, 0]),
            np.max(rectangle[:, 1]),
        )
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
        objects.append(bbox)

    # for object_id, centroid in tracker.update(objects).items():
    #     text = "ID {}".format(object_id)
    #     cv2.putText(
    #         frame,
    #         text,
    #         (centroid[0] - 10, centroid[1] - 10),
    #         cv2.FONT_HERSHEY_SIMPLEX,
    #         0.5,
    #         (0, 255, 0),
    #         2,
    #     )
    #     cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

    out_queue.put(frame)

    # zero_mask = fg_mask == 0
    # frame[zero_mask] = frame[zero_mask] // 2
    # cv2.polylines(frame, [polygon_roi], True, (0, 0, 255), 1)
    # frame = cv2.drawContours(
    #     frame,
    #     contours,
    #     -1,
    #     # color=colors[index % (len(colors) - 1)],
    #     color=(255, 255, 0),
    #     thickness=2,
    # )


def main():
    rospy.init_node("centroid_tracking_playground")
    window_name = "Frame"

    back_subtractor = cv2.createBackgroundSubtractorMOG2()
    tracker = CentroidTracker(max_disappeared=5)
    cv2.namedWindow(window_name)

    data = AppData(
        window_name=window_name,
        tracker=tracker,
        back_subtractor=back_subtractor,
    )

    rospy.Subscriber("/camera_1/image_raw", Image, lambda m: image_callback(data, m), queue_size=1)

    while True:
        keyboard = chr(cv2.waitKey(1) & 0xFF)
        if keyboard == "q":
            break

        if rospy.is_shutdown():
            break
        if data.out_queue.empty():
            rospy.sleep(0.1)
            continue

        frame = data.out_queue.get()

        cv2.imshow(window_name, frame)


main()
