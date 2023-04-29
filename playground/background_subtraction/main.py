import cv2
import imutils
import numpy as np
import argparse
from imutils.video import FileVideoStream

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
args = parser.parse_args()
if args.algo == "MOG2":
    backSub = cv2.createBackgroundSubtractorMOG2(
        history=100, varThreshold=32, detectShadows=False
    )
elif args.algo == "KNN":
    backSub = cv2.createBackgroundSubtractorKNN()
elif args.algo == "GMG":
    backSub = cv2.bgsegm.createBackgroundSubtractorGMG()
else:
    raise ValueError(f"Invalid algorithm supplied: {args.algo}")

contour_min_area = 200.0

capture = FileVideoStream(args.input).start()
frame_count = 0

while True:
    frame = capture.read()
    if frame is None:
        break
    frame = imutils.resize(frame, width=640)

    fgMask = backSub.apply(frame)
    fgMask = cv2.GaussianBlur(fgMask, (3, 3), 0)
    contours, hierarchy = cv2.findContours(
        fgMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    for contour in contours:
        if cv2.contourArea(contour) > contour_min_area:
            rectangle = cv2.minAreaRect(contour)
            x0, y0 = rectangle[0]
            width, height = rectangle[1]
            pt1 = (int(x0 - width / 2), int(y0 - height / 2))
            pt2 = (int(x0 + width / 2), int(y0 + height / 2))
            cv2.rectangle(frame, pt1, pt2, (255, 0, 0), 1)

    cv2.rectangle(frame, (10, 2), (100, 20), (255, 255, 255), -1)
    cv2.putText(
        frame,
        str(frame_count),
        (15, 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 0),
    )

    cv2.imshow("Frame", frame)
    cv2.imshow("FG Mask", fgMask)

    keyboard = cv2.waitKey(1)
    if keyboard == "q" or keyboard == 27:
        break
    frame_count += 1
