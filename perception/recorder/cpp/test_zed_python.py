import time

import cv2
from recorder_cpp import ZEDCamera

zed = ZEDCamera()
if not zed.open():
    print("Failed to open camera:", zed.get_last_error())
    exit(1)

print("Grabbing frames...")

frame = None
for i in range(5):
    if zed.grab():
        print(f"Frame {zed.get_frame_count()} grabbed")
        frame = zed.retrieve_image()
        cv2.imshow("ZED Frame", frame)
        cv2.waitKey(1)
    else:
        print("Grab failed:", zed.get_last_error())
    time.sleep(0.001)

if frame:
    cv2.imwrite("zed_frame.png", frame)
    print("Saved frame to zed_frame.png")

zed.close()
print("Camera closed.")
