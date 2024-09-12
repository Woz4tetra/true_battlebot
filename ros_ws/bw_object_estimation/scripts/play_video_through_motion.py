import argparse
import time

import cv2
import rospy
import tqdm
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main() -> None:
    rospy.init_node("play_video_through_motion")

    parser = argparse.ArgumentParser()
    parser.add_argument("video_file", help="Path to the bag file")
    parser.add_argument("-s", "--start-time", type=float, default=0.0, help="Start time in seconds")
    args = parser.parse_args()

    image_pub = rospy.Publisher("/camera_1/image_rect", Image, queue_size=10)
    cv_bridge = CvBridge()

    cv2.namedWindow("video")
    video = cv2.VideoCapture(args.video_file)
    num_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = int(video.get(cv2.CAP_PROP_FPS))

    def get_real_time() -> float:
        return time.perf_counter() - start_time

    start_frame = int(fps * args.start_time)
    if args.start_time > 0:
        video.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    try:
        with tqdm.tqdm(total=num_frames) as pbar:
            pbar.update(start_frame)
            start_time = time.perf_counter()
            for i in range(num_frames):
                pbar.update(1)
                success, frame = video.read()
                if not success:
                    print("Video ended")
                    break
                cv2.imshow("video", frame)
                key = chr(cv2.waitKey(1) & 0xFF)
                if key == "q":
                    break

                msg = cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                image_pub.publish(msg)

                video_time = i / fps
                real_time = get_real_time()

                if video_time > real_time:
                    rospy.sleep(video_time - real_time)

                if rospy.is_shutdown():
                    break
    finally:
        video.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
