#!/usr/bin/env python
import argparse
import math
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("videos", type=str, nargs="+", help="Paths to videos")
    parser.add_argument("-o", "--output", type=str, default="", help="Path to the output directory")
    parser.add_argument("-i", "--start-index", type=int, default=0, help="Start video at index")
    parser.add_argument("-s", "--skip-frames", type=int, default=0, help="Frame skip interval")
    parser.add_argument("-w", "--width", type=int, default=1920, help="Width of the output video")
    parser.add_argument("-n", "--no-show", action="store_true", help="Do not display the video")
    args = parser.parse_args()

    video_paths = [Path(path) for path in args.videos]
    first_video_path = video_paths[0]
    default_path = first_video_path.parent / Path(f"{first_video_path.stem}_tiled.avi")
    output_path = Path(args.output) if args.output else default_path
    skip_frames = args.skip_frames
    start_index = args.start_index
    output_width = int(args.width)
    show = not args.no_show

    for video_path in video_paths:
        if not video_path.is_file():
            raise FileNotFoundError(f"Video not found: {video_path}")

    video_names = [video_path.stem for video_path in video_paths]
    in_videos = [cv2.VideoCapture(str(video_path)) for video_path in video_paths]
    out_video: cv2.VideoWriter | None = None
    in_video_fpses = [in_video.get(cv2.CAP_PROP_FPS) for in_video in in_videos]
    out_video_fps = max(in_video_fpses) / (1 + skip_frames)
    in_num_frames = [int(in_video.get(cv2.CAP_PROP_FRAME_COUNT)) for in_video in in_videos]
    num_frames = max(in_num_frames)
    video_dims = [
        (int(in_video.get(cv2.CAP_PROP_FRAME_HEIGHT)), int(in_video.get(cv2.CAP_PROP_FRAME_WIDTH)))
        for in_video in in_videos
    ]

    for in_video in in_videos:
        in_video.set(cv2.CAP_PROP_POS_FRAMES, start_index)
    frame_num = start_index

    num_videos = len(in_videos)
    num_columns = math.ceil(math.sqrt(num_videos))
    num_rows = math.ceil(num_videos / num_columns)
    height_grid = (
        np.array([height for height, _ in video_dims] + [0] * ((num_columns * num_rows) - num_videos))
        .reshape(num_rows, num_columns)
        .astype(int)
    )
    height_grid = height_grid // num_columns
    row_heights = np.max(height_grid, axis=1)
    output_height = int(np.sum(row_heights))

    window_name = "Tiled Videos"
    if show:
        cv2.namedWindow(window_name)

    try:
        with tqdm(total=num_frames) as pbar:
            pbar.update(start_index)
            while True:
                pbar.update(1)
                frames = []
                if skip_frames > 0 and frame_num % skip_frames != 0:
                    frame_num += 1
                    continue
                all_finished = True
                for video_index, (video_count, in_video) in enumerate(zip(in_num_frames, in_videos)):
                    height, width = video_dims[video_index]
                    if frame_num >= video_count:
                        frame = np.zeros((height, width, 3), dtype=np.uint8)
                    else:
                        success, frame = in_video.read()  # type: ignore
                        if success:
                            all_finished = False
                        else:
                            frame = np.zeros((height, width, 3), dtype=np.uint8)

                    cv2.putText(
                        frame,
                        video_names[video_index],
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    frames.append(frame)
                if all_finished:
                    break

                combined_frame = np.zeros((output_height, output_width, 3), dtype=np.uint8)
                for video_index, frame in enumerate(frames):
                    row = video_index // num_columns
                    col = video_index % num_columns
                    y_start = row * output_height // num_rows
                    y_end = y_start + row_heights[row]
                    x_start = col * output_width // num_columns
                    x_end = (col + 1) * output_width // num_columns
                    frame = cv2.resize(frame, (x_end - x_start, y_end - y_start)).astype(np.uint8)
                    combined_frame[y_start:y_end, x_start:x_end] = frame

                if show:
                    cv2.imshow(window_name, combined_frame)
                    key = chr(cv2.waitKey(1) & 0xFF)
                    if key == "q":
                        break

                if out_video is None:
                    out_video = cv2.VideoWriter(
                        str(output_path),
                        cv2.VideoWriter_fourcc(*"XVID"),  # type: ignore
                        out_video_fps,
                        (output_width, output_height),
                    )
                out_video.write(combined_frame)
                frame_num += 1
    finally:
        for in_video in in_videos:
            in_video.release()
        if out_video is not None:
            out_video.release()


if __name__ == "__main__":
    main()
