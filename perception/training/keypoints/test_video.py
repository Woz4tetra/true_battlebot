import argparse
import time
from pathlib import Path

import cv2
import numpy as np
from perception_tools.config.model_metadata import ModelMetadata
from perception_tools.inference.common import load_metadata
from tqdm import tqdm
from ultralytics import YOLO


def draw_keypoints(result, metadata: ModelMetadata) -> np.ndarray:
    ids = result.boxes.cpu().cls.int().numpy()  # get the class ids
    keypoints = result.keypoints.cpu().xy.int().numpy()  # get the keypoints
    labels = [result.names[index] for index in ids]
    img_array = result.plot(kpt_line=True, kpt_radius=6)  # plot a BGR array of predictions

    # render keypoint names in image
    for keypoint, label in zip(keypoints, labels):
        keypoint = keypoint.tolist()
        front_label = metadata.keypoint_map[label][0]
        back_label = metadata.keypoint_map[label][1]
        cv2.putText(
            img_array,
            front_label,
            (keypoint[0][0], keypoint[0][1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            img_array,
            back_label,
            (keypoint[1][0], keypoint[1][1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
    return img_array


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("video", help="Path to video")
    parser.add_argument(
        "models",
        type=str,
        nargs="+",
        help="Path to the model(s) (*.pth or *.torchscript)",
    )
    parser.add_argument("-o", "--output", type=str, default="", help="Path to the output directory")
    parser.add_argument("-m", "--metadata", type=str, default="", help="Path to the metadata file (*.json)")
    parser.add_argument("-t", "--threshold", type=float, default=0.4, help="Confidence threshold for predictions")
    parser.add_argument("-i", "--start-index", type=int, default=0, help="Start video at index")
    parser.add_argument("-s", "--skip-frames", type=int, default=0, help="Frame skip interval")
    parser.add_argument("-n", "--no-show", action="store_true", help="Do not display the video")
    args = parser.parse_args()

    model_paths = [Path(path) for path in args.models]
    video_path = Path(args.video)
    metadata_path = args.metadata
    threshold = args.threshold
    start_index = args.start_index
    skip_frames = args.skip_frames
    show = not args.no_show
    is_metadata_set = bool(metadata_path)

    window_name = str(video_path.stem)
    if show:
        cv2.namedWindow(window_name)

    should_quit = False

    for model_path in model_paths:
        default_path = video_path.parent / Path(f"{video_path.stem}_{model_path.stem}.avi")
        output_path = Path(args.output) if args.output else default_path

        print(f"Loading YOLO model: {model_path}")
        model = YOLO(model_path, task="pose")

        if not is_metadata_set:
            metadata_path = model_path.with_suffix(".json")
        print(f"Loading metadata from {metadata_path}")
        metadata = load_metadata(metadata_path)

        if not video_path.is_file():
            raise FileNotFoundError(f"Video not found: {video_path}")

        assert skip_frames >= 0, "Skip frames must be greater than or equal to 0"

        in_video = cv2.VideoCapture(str(video_path))  # type: ignore
        out_video: cv2.VideoWriter | None = None
        video_fps = in_video.get(cv2.CAP_PROP_FPS) / (1 + skip_frames)
        num_frames = int(in_video.get(cv2.CAP_PROP_FRAME_COUNT))

        in_video.set(cv2.CAP_PROP_POS_FRAMES, start_index)
        frame_num = start_index
        diffs = []

        try:
            with tqdm(total=num_frames) as pbar:
                pbar.update(start_index)
                while True:
                    success, frame = in_video.read()
                    pbar.update(1)
                    if not success:
                        print("Video ended")
                        break
                    frame_num += 1
                    if skip_frames > 0 and frame_num % skip_frames != 0:
                        continue
                    if out_video is None:
                        print(f"Saving output to {output_path}")
                        height, width = frame.shape[:2]
                        out_video = cv2.VideoWriter(
                            str(output_path),
                            cv2.VideoWriter_fourcc(*"XVID"),  # type: ignore
                            video_fps,
                            (width, height),
                        )

                    t0 = time.perf_counter()
                    results = model(frame, verbose=False, conf=threshold)[0]
                    t1 = time.perf_counter()
                    delta = t1 - t0
                    diffs.append(delta)
                    debug_image = draw_keypoints(results, metadata)

                    if show:
                        cv2.imshow(window_name, debug_image)
                        key = chr(cv2.waitKey(1) & 0xFF)
                        if key == "q":
                            should_quit = True
                            break
                    if out_video is not None:
                        out_video.write(debug_image)
        finally:
            in_video.release()
            if out_video is not None:
                out_video.release()

            if len(diffs) > 0:
                print(f"{model_path.stem} Warmup time: {diffs.pop(0):0.6f} s")
            if len(diffs) > 0:
                print(f"{model_path.stem} Average inference rate: {len(diffs) / sum(diffs):0.6f} fps")
            if should_quit:
                break


if __name__ == "__main__":
    main()
