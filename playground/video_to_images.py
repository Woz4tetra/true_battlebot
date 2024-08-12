import argparse
import os

import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=str, nargs="+", help="path to input video file(s)")
    parser.add_argument("-o", "--output", type=str, default="", help="output directory")
    parser.add_argument("-s", "--skip", type=int, default=0, help="select x number of frames")
    parser.add_argument("-l", "--left", action="store_true", help="extract left half of the video")
    args = parser.parse_args()

    frame_skip = args.skip
    split_left = args.left

    for video_name in args.video:
        video_path = os.path.abspath(video_name)
        if len(args.output) == 0:
            filename = os.path.basename(video_path)
            output_dir = os.path.join(os.path.dirname(video_path), os.path.splitext(filename)[0])
        print(f"Output directory: {output_dir}")

        if os.path.isfile(output_dir):
            raise FileExistsError("Output directory is a file!")

        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        video_name = os.path.splitext(os.path.basename(video_path))[0]
        video = cv2.VideoCapture(video_path)
        frame_count = 0
        while True:
            if frame_skip > 0:
                frame_count += frame_skip
                video.set(cv2.CAP_PROP_POS_FRAMES, frame_count)
            else:
                frame_count += 1
            success, frame = video.read()
            if not success:
                print("Video finished")
                break
            if split_left:
                frame = frame[:, : frame.shape[1] // 2]
            image_name = f"{video_name}-{frame_count:06d}.jpg"
            image_path = os.path.join(output_dir, image_name)
            print(f"Writing to {image_path}")
            cv2.imwrite(image_path, frame)


main()
