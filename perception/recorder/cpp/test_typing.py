#!/usr/bin/env python3
"""
Test script to verify type hints work with mypy.
Run with: mypy test_typing.py
"""

from recorder_cpp import ZEDCamera


def test_camera_typing() -> None:
    """Test that type hints are working correctly."""
    # Create camera instance
    camera: ZEDCamera = ZEDCamera()

    # Test method return types
    opened: bool = camera.open()
    if not opened:
        error: str = camera.get_last_error()
        print(f"Failed to open camera: {error}")
        return

    # Test streaming
    streaming_enabled: bool = camera.enable_streaming(30000)
    if streaming_enabled:
        # Grab some frames
        for _ in range(5):
            success: bool = camera.grab()
            if success:
                frame_count: int = camera.get_frame_count()
                print(f"Frame count: {frame_count}")

    # Cleanup
    camera.disable_streaming()
    camera.close()


if __name__ == "__main__":
    test_camera_typing()
