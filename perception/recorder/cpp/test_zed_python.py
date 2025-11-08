import time

from recorder_cpp import ZEDCamera

zed = ZEDCamera()
if not zed.open():
    print("Failed to open camera:", zed.get_last_error())
    exit(1)

print("Grabbing frames...")

for i in range(5):
    if zed.grab():
        print(f"Frame {zed.get_frame_count()} grabbed")
    else:
        print("Grab failed:", zed.get_last_error())
    time.sleep(0.001)

zed.close()
print("Camera closed.")
