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
        
        # Video trimming
        self.trim_start = 0
        self.trim_end = self.total_frames - 1
        self.trim_mode = False
        
        print(f"Video: {video_path}")
        print(f"Frames: {self.total_frames}, FPS: {self.fps:.2f}")
        print(f"Resolution: {self.frame_width}x{self.frame_height}")
        print(f"Duration: {self.total_frames/self.fps:.2f} seconds")
        if start_time > 0:
            print(f"Starting at: {start_time:.2f}s (frame {self.start_frame})")
        
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks to add points."""
        if event == cv2.EVENT_LBUTTONDOWN:
            # Add point at current frame
            self.points.append((self.current_frame, x, y, self.point_counter))
            self.point_counter += 1
            print(f"Added point {self.point_counter} at frame {self.current_frame}: ({x}, {y})")
            self.draw_frame()
            
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Remove nearest point if any exists
            if self.points:
                # Find nearest point
                min_dist = float('inf')
                nearest_idx = -1
                for i, (frame_idx, px, py, point_id) in enumerate(self.points):
                    if frame_idx == self.current_frame:
                        dist = np.sqrt((x - px)**2 + (y - py)**2)
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
            cv2.putText(display_frame, str(point_id), (x + 10, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Draw frame info
        info_text = f"Frame: {self.current_frame}/{self.total_frames-1} | Points: {len(current_frame_points)} | Total Points: {len(self.points)}"
        cv2.putText(display_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)
        
        # Draw trim range info
        trim_duration = (self.trim_end - self.trim_start) / self.fps
        trim_info = f"Trim: {self.trim_start}-{self.trim_end} ({trim_duration:.1f}s)"
        if self.trim_mode:
            trim_info += " [TRIM MODE - T to toggle]"
        cv2.putText(display_frame, trim_info, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display_frame, trim_info, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if not self.trim_mode else (0, 255, 255), 1)
        
        # Draw trim markers
        if self.current_frame == self.trim_start:
            cv2.rectangle(display_frame, (0, 0), (self.frame_width, self.frame_height), (0, 255, 0), 3)
            cv2.putText(display_frame, "START", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        elif self.current_frame == self.trim_end:
            cv2.rectangle(display_frame, (0, 0), (self.frame_width, self.frame_height), (0, 0, 255), 3)
            cv2.putText(display_frame, "END", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Draw controls
        controls = [
            "Left Click: Add Point | Right Click: Remove Point",
            "SPACE: Play/Pause | A/D: Previous/Next Frame | T: Trim Mode",
            "S: Save & Export Trimmed | E: Export Trimmed Video | Q: Quit",
            "1-9: Jump to 10%-90% | 0: Jump to start"
        ]
        
        for i, control in enumerate(controls):
            y_pos = self.frame_height - 60 + i * 20
            cv2.putText(display_frame, control, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(display_frame, control, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        cv2.imshow(self.window_name, display_frame)
    
    def get_color(self, point_id: int) -> Tuple[int, int, int]:
        """Get a unique color for each point ID."""
        colors = [
            (255, 0, 0),    # Red
            (0, 255, 0),    # Green  
            (0, 0, 255),    # Blue
            (255, 255, 0),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Yellow
            (128, 0, 128),  # Purple
            (255, 165, 0),  # Orange
            (0, 128, 128),  # Teal
            (128, 128, 128) # Gray
        ]
        return colors[point_id % len(colors)]
    
    def play_video(self):
        """Play the video automatically."""
        while self.playing and self.current_frame < self.total_frames - 1:
            if self.load_frame(self.current_frame + 1):
                self.draw_frame()
                key = cv2.waitKey(int(1000/self.fps)) & 0xFF
                if key != 255:  # Any key pressed
                    self.playing = False
                    self.handle_key(key)
            else:
                self.playing = False
    
    def handle_key(self, key: int) -> bool:
        """Handle keyboard input. Returns False to quit."""
        if key == ord('q') or key == 27:  # Q or ESC
            return False
            
        elif key == ord(' '):  # SPACE - Play/Pause
            self.playing = not self.playing
            if self.playing:
                print("Playing...")
                self.play_video()
            else:
                print("Paused")
                
        elif key == ord('a'):  # A - Previous frame
            if self.current_frame > 0:
                self.load_frame(self.current_frame - 1)
                self.draw_frame()
                
        elif key == ord('d'):  # D - Next frame
            if self.current_frame < self.total_frames - 1:
                self.load_frame(self.current_frame + 1)
                self.draw_frame()
                
        elif key == ord('t'):  # T - Toggle trim mode
            self.trim_mode = not self.trim_mode
            if self.trim_mode:
                print("Trim mode ON - Left/Right arrow keys set trim start/end")
            else:
                print("Trim mode OFF")
                
        elif key == ord('s'):  # S - Save CSV and export trimmed video
            self.save_points()
            self.export_trimmed_video()
            
        elif key == ord('e'):  # E - Export trimmed video only
            self.export_trimmed_video()
            
        elif key == 81 or key == 2:  # Left arrow - Set trim start (in trim mode)
            if self.trim_mode:
                self.trim_start = self.current_frame
                print(f"Trim start set to frame {self.trim_start}")
                self.draw_frame()
            else:
                # Regular left arrow behavior (jump back 10 frames)
                target = max(0, self.current_frame - 10)
                self.load_frame(target)
                self.draw_frame()
                
        elif key == 83 or key == 3:  # Right arrow - Set trim end (in trim mode)
            if self.trim_mode:
                self.trim_end = self.current_frame
                print(f"Trim end set to frame {self.trim_end}")
                self.draw_frame()
            else:
                # Regular right arrow behavior (jump forward 10 frames)
                target = min(self.total_frames - 1, self.current_frame + 10)
                self.load_frame(target)
                self.draw_frame()
            
        elif key >= ord('0') and key <= ord('9'):  # 0-9 - Jump to percentage
            percent = (key - ord('0')) * 10
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
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['point_id', 'frame', 'x', 'y'])  # Header
            
            for point_id in sorted(point_groups.keys()):
                frame_idx, x, y = point_groups[point_id]
                writer.writerow([point_id, frame_idx, x, y])
        
        print(f"Saved {len(point_groups)} points to {csv_path}")
        
        # Also save detailed version with all point instances
        detailed_csv_path = f"{video_name}_points_detailed.csv"
        with open(detailed_csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['point_id', 'frame', 'x', 'y'])  # Header
            
            # Sort by point_id, then by frame
            sorted_points = sorted(self.points, key=lambda p: (p[3], p[0]))
            for frame_idx, x, y, point_id in sorted_points:
                writer.writerow([point_id, frame_idx, x, y])
        
        print(f"Saved detailed points (all instances) to {detailed_csv_path}")
    
    def export_trimmed_video(self):
        """Export trimmed video with adjusted point coordinates."""
        if self.trim_start >= self.trim_end:
            print("Invalid trim range. Start must be less than end.")
            return
            
        # Generate output filename
        video_name = os.path.splitext(os.path.basename(self.video_path))[0]
        trimmed_video_path = f"{video_name}_trimmed.mp4"
        
        print(f"Exporting trimmed video: frames {self.trim_start}-{self.trim_end}")
        print(f"Output: {trimmed_video_path}")
        
        # Set up video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(trimmed_video_path, fourcc, self.fps, (self.frame_width, self.frame_height))
        
        # Reset capture to start frame
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.trim_start)
        
        frames_to_export = self.trim_end - self.trim_start + 1
        frames_exported = 0
        
        print(f"Exporting {frames_to_export} frames...")
        
        for frame_idx in range(self.trim_start, self.trim_end + 1):
            ret, frame = self.cap.read()
            if not ret:
                print(f"Warning: Could not read frame {frame_idx}")
                break
                
            out.write(frame)
            frames_exported += 1
            
            # Progress indicator
            if frames_exported % 30 == 0:  # Every 30 frames
                progress = frames_exported / frames_to_export * 100
                print(f"Progress: {progress:.1f}%")
        
        out.release()
        
        print(f"Exported {frames_exported} frames to {trimmed_video_path}")
        
        # Also save adjusted CSV if points exist
        if self.points:
            self.save_adjusted_points_csv(trimmed_video_path)
    
    def save_adjusted_points_csv(self, trimmed_video_path):
        """Save CSV with frame numbers adjusted for trimmed video."""
        # Generate CSV filename based on trimmed video name
        video_name = os.path.splitext(os.path.basename(trimmed_video_path))[0]
        csv_path = f"{video_name}_points.csv"
        
        # Filter and adjust points
        adjusted_points = {}
        for frame_idx, x, y, point_id in self.points:
            if self.trim_start <= frame_idx <= self.trim_end:
                # Adjust frame number to be relative to trim start
                adjusted_frame = frame_idx - self.trim_start
                if point_id not in adjusted_points:
                    adjusted_points[point_id] = (adjusted_frame, x, y)
        
        if not adjusted_points:
            print("No points within trim range to save.")
            return
            
        # Write adjusted CSV
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['point_id', 'frame', 'x', 'y'])  # Header
            
            for point_id in sorted(adjusted_points.keys()):
                frame_idx, x, y = adjusted_points[point_id]
                writer.writerow([point_id, frame_idx, x, y])
        
        print(f"Saved adjusted points (for trimmed video) to {csv_path}")
        print(f"Frame numbers adjusted: original range {self.trim_start}-{self.trim_end} → 0-{self.trim_end-self.trim_start}")
    
    def run(self):
        """Main labeling loop."""
        print("\nStarting video labeler with trimming...")
        print("Controls:")
        print("  Left Click: Add point at current frame")
        print("  Right Click: Remove nearest point")
        print("  SPACE: Play/Pause video")
        print("  A/D: Previous/Next frame")
        print("  T: Toggle trim mode")
        print("  Left/Right arrows: Set trim start/end (in trim mode) or jump ±10 frames")
        print("  S: Save points CSV + export trimmed video")
        print("  E: Export trimmed video only")
        print("  0-9: Jump to 0%-90% of video")
        print("  Q/ESC: Quit")
        print()
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
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
            if save_choice == 'y':
                self.save_points()


def main():
    parser = argparse.ArgumentParser(description="Interactive video point labeling tool for TAPIR")
    parser.add_argument("video_path", help="Path to input video file")
    parser.add_argument("--start-time", "-t", type=float, default=0.0,
                        help="Start time in seconds (default: 0.0)")
    
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