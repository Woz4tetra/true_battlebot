import argparse
import csv
import os

import cv2
import matplotlib
import numpy as np
from matplotlib import pyplot as plt

matplotlib.use("Agg")


def sum_frame_deltas(frames: list[np.ndarray]) -> np.ndarray:
    area = np.prod(frames[0].shape[:2])
    sums = np.array([float(np.sum(cv2.absdiff(frames[i - 1], frames[i]))) for i in range(1, len(frames))])
    return sums / area


def find_nearest_index(array: list[float], value: float) -> int:
    return int(np.argmin(np.abs(np.array(array) - value)))


def get_summary_stats(delays: list[float]) -> tuple[float, float, float, float]:
    if not delays:
        return (0.0, 0.0, 0.0, 0.0)
    return (float(min(delays)), float(max(delays)), float(np.mean(delays)), float(np.std(delays)))


def print_summary_stats(start_delays: list[float], stop_delays: list[float]) -> None:
    if not start_delays or not stop_delays:
        print("No delays")
        return
    header = "N\tStart Min\tStart Max\tStart Mean\tStart Std dev\tStop Min\tStop Max\tStop Mean\tStop Std dev"
    print(header)
    print("-" * len(header))
    print(len(start_delays), end="\t")
    start_stats_str = "\t".join(map(str, get_summary_stats(start_delays)))
    stop_stats_str = "\t".join(map(str, get_summary_stats(stop_delays)))
    print(f"{start_stats_str}\t{stop_stats_str}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("data_path", type=str)
    args = parser.parse_args()

    data_path = args.data_path
    video_path = os.path.splitext(data_path)[0] + ".avi"
    data_dir = os.path.dirname(data_path)
    start_commands = []
    stop_commands = []
    frame_times = []

    with open(data_path, "r") as data_file:
        for row in csv.reader(data_file):
            timestamp = float(row[0])
            key = row[1]
            if key == "command":
                command = [float(row[2]), float(row[3])]
                if any(command):
                    start_commands.append(timestamp)
                else:
                    stop_commands.append(timestamp)
            elif key == "frame":
                frame_times.append(timestamp)

    video = cv2.VideoCapture(video_path)
    select_frame = None
    count = 0
    while True:
        success, frame = video.read()
        if success:
            select_frame = frame
        if count > 50 or not success:
            break
    if select_frame is None:
        raise Exception("Failed to read video frame")
    roi = cv2.selectROI("Image", select_frame)
    if roi is None:
        raise Exception("Failed to select ROI")
    cv2.destroyAllWindows()
    frames = []
    video.set(cv2.CAP_PROP_POS_FRAMES, 0)

    frame_num = 0
    while True:
        success, frame = video.read()
        if not success:
            break
        cropped = frame[int(roi[1]) : int(roi[1] + roi[3]), int(roi[0]) : int(roi[0] + roi[2])]
        frame_num += 1
        frames.append(cropped)

    ignore_start_time = 0.5

    ignore_start_index = find_nearest_index(frame_times, start_commands[0] + ignore_start_time)
    frames = frames[ignore_start_index:]
    frame_times = frame_times[ignore_start_index:]
    start_commands = [start_command for start_command in start_commands if start_command > frame_times[0]]
    stop_commands = [stop_command for stop_command in stop_commands if stop_command > frame_times[0]]

    frame_deltas = sum_frame_deltas(frames)

    start_motion_threshold = 42.0
    stop_motion_threshold = 16.0

    while True:
        plt.plot(frame_times[1:], frame_deltas)
        plt.axhline(start_motion_threshold, color="lightcoral")
        plt.axhline(stop_motion_threshold, color="darkturquoise")
        for start_command in start_commands:
            plt.axvline(start_command, color="g")
        for stop_command in stop_commands:
            plt.axvline(stop_command, color="b")
        plt.savefig(os.path.join(data_dir, "frame_deltas.png"))
        plt.clf()

        start_delays = []
        for start_command in start_commands:
            first_index = find_nearest_index(frame_times, start_command) + 1
            motion_mask = frame_deltas[first_index:] > start_motion_threshold
            if not np.any(motion_mask):
                print(f"No motion detected for start command at {start_command}")
            else:
                start_frame = np.where(motion_mask)[0][0]
                delay = frame_times[first_index + start_frame] - start_command
                start_delays.append(delay)

        stop_delays = []
        for stop_command in stop_commands:
            first_index = find_nearest_index(frame_times, stop_command) + 1
            motion_mask = frame_deltas[first_index:] < stop_motion_threshold
            if not np.any(motion_mask):
                print(f"No motion detected for stop command at {stop_command}")
            else:
                stop_frame = np.where(motion_mask)[0][0]
                delay = frame_times[first_index + stop_frame] - stop_command
                stop_delays.append(delay)

        print_summary_stats(start_delays, stop_delays)

        command = input("Enter to continue, 'q' to exit: ")
        if command == "q":
            return

        start_str = input("Enter start threshold: ")
        try:
            start_motion_threshold = float(start_str)
        except ValueError:
            print("Invalid input")
            continue

        stop_str = input("Enter stop threshold: ")
        try:
            stop_motion_threshold = float(stop_str)
        except ValueError:
            print("Invalid input")
            continue


if __name__ == "__main__":
    main()
