#!/usr/bin/env python
"""
Interactive point labeling tool for TAPIR tracking.
Creates a CSV file of manually selected points that can be used with tapnet_auto_annotate_video.py.
"""

import argparse
import csv
import os
from typing import List, Tuple

import cv2
import numpy as np


class VideoLabeler:
    """Interactive video labeling tool for selecting points to track."""

    def __init__(self, video_path: str, start_time: float = 0.0):
        self.video_path = video_path
        self.cap = cv2.VideoCapture(video_path)

        if not self.cap.isOpened():
            raise ValueError(f"Could not open video: {video_path}")

        # Get video properties
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Calculate start frame from start time
        self.start_frame = int(start_time * self.fps)
        self.start_frame = max(0, min(self.start_frame, self.total_frames - 1))

        # State variables
        self.current_frame = self.start_frame
        self.current_image = None
        self.points = []  # List of (frame_idx, x, y, point_id) tuples
        self.point_counter = 0
        self.playing = False
        self.window_name = "Video Labeler"

        print(f"Video: {video_path}")
        print(f"Frames: {self.total_frames}, FPS: {self.fps:.2f}")
        print(f"Resolution: {self.frame_width}x{self.frame_height}")
        print(f"Duration: {self.total_frames / self.fps:.2f} seconds")
        if start_time > 0:
            print(f"Starting at: {start_time:.2f}s (frame {self.start_frame})")

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks to add points."""
        if event == cv2.EVENT_LBUTTONDOWN:
            # Add point at current frame
            self.points.append((self.current_frame, x, y, self.point_counter))
            print(f"Added point {self.point_counter} at frame {self.current_frame}: ({x}, {y})")
            self.point_counter += 1
            self.draw_frame()

        elif event == cv2.EVENT_RBUTTONDOWN:
            # Remove nearest point if any exists
            if self.points:
                # Find nearest point
                min_dist = float("inf")
                nearest_idx = -1
                for i, (frame_idx, px, py, point_id) in enumerate(self.points):
                    if frame_idx == self.current_frame:
                        dist = np.sqrt((x - px) ** 2 + (y - py) ** 2)
                        if dist < min_dist and dist < 20:  # Within 20 pixels
                            min_dist = dist
                            nearest_idx = i

                if nearest_idx >= 0:
                    removed = self.points.pop(nearest_idx)
                    print(f"Removed point {removed[3]} at frame {removed[0]}: ({removed[1]}, {removed[2]})")
                    self.draw_frame()

    def load_frame(self, frame_idx: int) -> bool:
        """Load a specific frame from the video."""
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = self.cap.read()
        if ret:
            self.current_frame = frame_idx
            self.current_image = frame.copy()
            return True
        return False

    def draw_frame(self):
        """Draw the current frame with overlaid points."""
        if self.current_image is None:
            return

        display_frame = self.current_image.copy()

        # Draw all points for current frame
        current_frame_points = [p for p in self.points if p[0] == self.current_frame]

        for frame_idx, x, y, point_id in current_frame_points:
            # Draw point as colored circle
            color = self.get_color(point_id)
            cv2.circle(display_frame, (x, y), 5, color, -1)
            cv2.circle(display_frame, (x, y), 7, (255, 255, 255), 1)  # White border
            # Draw point ID
            cv2.putText(display_frame, str(point_id), (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Draw frame info
        info_text = f"Frame: {self.current_frame}/{self.total_frames - 1} | Points: {len(current_frame_points)} | Total Points: {len(self.points)}"
        cv2.putText(display_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)

        # Draw controls
        controls = [
            "Left Click: Add Point | Right Click: Remove Point",
            "SPACE: Play/Pause | A/D: Previous/Next Frame | Left/Right: Jump ±10 frames",
            "S: Save Points CSV | Q: Quit",
            "1-9: Jump to 10%-90% | 0: Jump to start",
        ]

        for i, control in enumerate(controls):
            y_pos = self.frame_height - 60 + i * 20
            cv2.putText(display_frame, control, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(display_frame, control, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        cv2.imshow(self.window_name, display_frame)

    def get_color(self, point_id: int) -> Tuple[int, int, int]:
        """Get a unique color for each point ID."""
        colors = [
            (255, 0, 0),  # Red
            (0, 255, 0),  # Green
            (0, 0, 255),  # Blue
            (255, 255, 0),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Yellow
            (128, 0, 128),  # Purple
            (255, 165, 0),  # Orange
            (0, 128, 128),  # Teal
            (128, 128, 128),  # Gray
        ]
        return colors[point_id % len(colors)]

    def play_video(self):
        """Play the video automatically."""
        while self.playing and self.current_frame < self.total_frames - 1:
            if self.load_frame(self.current_frame + 1):
                self.draw_frame()
                key = cv2.waitKey(int(1000 / self.fps)) & 0xFF
                if key != 255:  # Any key pressed
                    self.playing = False
                    self.handle_key(key)
            else:
                self.playing = False

    def handle_key(self, key: int) -> bool:
        """Handle keyboard input. Returns False to quit."""
        if key == ord("q") or key == 27:  # Q or ESC
            return False

        elif key == ord(" "):  # SPACE - Play/Pause
            self.playing = not self.playing
            if self.playing:
                print("Playing...")
                self.play_video()
            else:
                print("Paused")

        elif key == ord("a"):  # A - Previous frame
            if self.current_frame > 0:
                self.load_frame(self.current_frame - 1)
                self.draw_frame()

        elif key == ord("d"):  # D - Next frame
            if self.current_frame < self.total_frames - 1:
                self.load_frame(self.current_frame + 1)
                self.draw_frame()

        elif key == ord("s"):  # S - Save CSV
            self.save_points()

        elif key == 81 or key == 2:  # Left arrow - Jump back 10 frames
            target = max(0, self.current_frame - 10)
            self.load_frame(target)
            self.draw_frame()

        elif key == 83 or key == 3:  # Right arrow - Jump forward 10 frames
            target = min(self.total_frames - 1, self.current_frame + 10)
            self.load_frame(target)
            self.draw_frame()

        elif key >= ord("0") and key <= ord("9"):  # 0-9 - Jump to percentage
            percent = (key - ord("0")) * 10
            if percent == 0:
                target_frame = 0
            else:
                target_frame = int((percent / 100.0) * (self.total_frames - 1))
            self.load_frame(target_frame)
            self.draw_frame()
            print(f"Jumped to {percent}% (frame {target_frame})")

        return True

    def save_points(self):
        """Save points to CSV file."""
        if not self.points:
            print("No points to save!")
            return

        # Generate CSV filename based on video name
        video_name = os.path.splitext(os.path.basename(self.video_path))[0]
        csv_path = f"{video_name}_points.csv"

        # Group points by point_id and save each point's first occurrence
        point_groups = {}
        for frame_idx, x, y, point_id in self.points:
            if point_id not in point_groups:
                point_groups[point_id] = (frame_idx, x, y)

        # Write to CSV
        with open(csv_path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["point_id", "frame", "x", "y"])  # Header

            for point_id in sorted(point_groups.keys()):
                frame_idx, x, y = point_groups[point_id]
                writer.writerow([point_id, frame_idx, x, y])

        print(f"Saved {len(point_groups)} points to {csv_path}")

    def run(self):
        """Main labeling loop."""
        print("\nStarting video labeler...")
        print("Controls:")
        print("  Left Click: Add point at current frame")
        print("  Right Click: Remove nearest point")
        print("  SPACE: Play/Pause video")
        print("  A/D: Previous/Next frame")
        print("  Left/Right arrows: Jump ±10 frames")
        print("  S: Save points CSV")
        print("  0-9: Jump to 0%-90% of video")
        print("  Q/ESC: Quit")
        print()

        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        # Load start frame
        self.load_frame(self.start_frame)
        self.draw_frame()

        # Main loop
        while True:
            key = cv2.waitKey(0) & 0xFF
            if not self.handle_key(key):
                break

        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()

        # Ask to save if points exist and not saved yet
        if self.points:
            print(f"\nYou have {len(self.points)} labeled points.")
            save_choice = input("Save points to CSV? (y/N): ").lower().strip()
            if save_choice == "y":
                self.save_points()


def main():
    parser = argparse.ArgumentParser(description="Interactive video point labeling tool for TAPIR")
    parser.add_argument("video_path", help="Path to input video file")
    parser.add_argument("--start-time", "-t", type=float, default=0.0, help="Start time in seconds (default: 0.0)")

    args = parser.parse_args()

    if not os.path.exists(args.video_path):
        print(f"Error: Video file not found: {args.video_path}")
        return

    if args.start_time < 0:
        print(f"Error: Start time must be >= 0, got {args.start_time}")
        return

    try:
        labeler = VideoLabeler(args.video_path, args.start_time)
        labeler.run()
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
