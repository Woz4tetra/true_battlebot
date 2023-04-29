import os
from centroid_tracker import CentroidTracker
from imutils.video import FileVideoStream
import numpy as np
import argparse
import imutils
import cv2
import csv
from poly_roi_selector import OrientedROISelector


def get_args():
    parser = argparse.ArgumentParser(
        description="This program shows how to use background subtraction methods provided by \
                                                Opencv2. You can process both videos and images."
    )
    parser.add_argument(
        "-i",
        "--input",
        type=str,
        help="Path to a video or a sequence of image.",
        default="vtest.avi",
    )
    parser.add_argument(
        "--algo",
        type=str,
        help="Background subtraction method (KNN, MOG2).",
        default="MOG2",
    )
    parser.add_argument(
        "--roi",
        default="roi.txt",
        type=str,
        help="ROI coords file",
    )
    return parser.parse_args()


def select_roi(window_name, frame):
    roi_selector = None

    roi = None
    while True:
        if frame is None:
            break
        keyboard = chr(cv2.waitKey(1) & 0xFF)
        if keyboard == "q":
            return
        if roi_selector is None:
            roi_selector = OrientedROISelector(frame, window_name)
            roi_selector.resetCanvas(frame)
        else:
            roi = roi_selector.ROIs
            if len(roi) != 0:
                break
    return roi


def write_roi(path, roi):
    with open(path, "w") as file:
        writer = csv.writer(file)
        print(roi)
        for row in roi:
            writer.writerow(row.tolist())


def read_roi(path):
    with open(path, "r") as file:
        reader = csv.reader(file)
        return np.array(list(reader), np.int32)


def main():
    min_area = 350.0
    resize_width = 800

    roi = None
    window_name = "Frame"

    args = get_args()
    back_sub = cv2.bgsegm.createBackgroundSubtractorGMG()
    capture = FileVideoStream(args.input).start()
    tracker = CentroidTracker(maxDisappeared=1000)
    cv2.namedWindow(window_name)

    # colors = (
    #     (255, 0, 0),
    #     (0, 255, 0),
    #     (0, 0, 255),
    #     (255, 255, 0),
    #     (255, 0, 255),
    #     (255, 0, 128),
    #     (0, 255, 128),
    #     (128, 0, 255),
    #     (128, 0, 255),
    #     (0, 128, 255),
    #     (255, 0, 128),
    # )

    frame = capture.read()
    frame = imutils.resize(frame, width=resize_width)
    if len(args.roi) == 0 or not os.path.isfile(args.roi):
        roi = select_roi(window_name, frame)
        assert roi is not None
        polygon_roi = roi[0]["Polygon"][0]
        write_roi(args.roi, polygon_roi)
    else:
        polygon_roi = read_roi(args.roi)

    roi_mask = np.zeros(frame.shape[0:2])
    cv2.fillPoly(roi_mask, [polygon_roi], (255,))

    while True:
        frame = capture.read()
        if frame is None:
            break

        frame = imutils.resize(frame, width=resize_width)
        fg_mask = back_sub.apply(frame)
        fg_mask[roi_mask == 0] = 0
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        sure_fg = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel, iterations=5)
        sure_fg = cv2.erode(sure_fg, kernel, iterations=7)
        sure_bg = cv2.dilate(fg_mask, kernel, iterations=7)
        unknown = cv2.subtract(sure_bg, sure_fg)

        ret, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1
        markers[unknown == 255] = 0

        markers = cv2.watershed(frame, markers)

        labels = np.unique(markers)

        zero_mask = fg_mask == 0
        frame[zero_mask] = frame[zero_mask] // 2

        objects = []
        for index, label in enumerate(labels[2:]):
            # Create a binary image in which only the area of the label is in the foreground
            # and the rest of the image is in the background
            target = np.where(markers == label, 255, 0).astype(np.uint8)

            # Perform contour extraction on the created binary image
            contours, hierarchy = cv2.findContours(
                target, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if cv2.contourArea(contours[0]) < min_area:
                continue
            rectangle = cv2.boxPoints(cv2.minAreaRect(contours[0]))
            rectangle = rectangle.astype(np.int32)
            bbox = (
                np.min(rectangle[:, 0]),
                np.min(rectangle[:, 1]),
                np.max(rectangle[:, 0]),
                np.max(rectangle[:, 1]),
            )
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            objects.append(bbox)

            # frame = cv2.drawContours(
            #     frame,
            #     [contours[0]],
            #     -1,
            #     # color=colors[index % (len(colors) - 1)],
            #     color=(255, 255, 0),
            #     thickness=2,
            # )

        for objectID, centroid in tracker.update(objects).items():
            # draw both the ID of the object and the centroid of the
            # object on the output frame
            text = "ID {}".format(objectID)
            cv2.putText(
                frame,
                text,
                (centroid[0] - 10, centroid[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

        cv2.polylines(frame, [polygon_roi], True, (0, 0, 255), 1)

        frame = imutils.resize(frame, width=1280)
        cv2.imshow(window_name, frame)

        keyboard = chr(cv2.waitKey(1) & 0xFF)
        if keyboard == "q":
            break


main()
