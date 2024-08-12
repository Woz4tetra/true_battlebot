# made at 2020-06-02 15:16:45
# by nngogol

import cv2 as cv
import numpy as np

cv2 = cv

# Name your window
root_wind = "main"
cv.namedWindow(root_wind)


def on_hello(*args):
    print("hello", args)


def on_bye(*args):
    print("bye", args)


def on_check(*args):
    print("checkbox", args)


def on_radio(*args):
    print("radio", args)


# simple button
cv.createButton("hello", on_hello, None, cv.QT_NEW_BUTTONBAR)
cv.createButton("hello", on_hello)
# userData
cv.createButton("bye1", on_bye, [1, "string"])
cv.createButton("bye2", on_bye, [[2, 3], "qwerty"])
cv.createButton("bye3", on_bye, [2, 3, 4, 5, 6])
cv.createButton("bye", on_hello, None, cv.QT_NEW_BUTTONBAR)

# checkbox
cv.createButton("checkbox", on_hello, None, cv.QT_NEW_BUTTONBAR)
cv.createButton("checkbox1", on_check, [40, 50], 1, 0)
cv.createButton("checkbox2", on_check, [40, 50], 1, 1)
# radio
cv.createButton("radio", on_hello, None, cv.QT_NEW_BUTTONBAR)
cv.createButton("r1", on_radio, 1, 2, 0)
cv.createButton("r2", on_radio, [2], 2, 1)
cv.createButton("r3", on_radio, [3, "hello"], 2, 0)


# Create a black image
img = np.ones((555, 555, 3), np.uint8)
img.fill(127)
while True:
    cv.imshow(root_wind, img)

    code = cv.waitKey(1)

    if code == ord("q"):
        break

cv.destroyAllWindows()
