"""Type stubs for recorder_cpp nanobind module"""

class ZEDCamera:
    """
    Python wrapper for ZED Camera with streaming capabilities.

    This class provides a simplified interface to the ZED SDK for camera
    initialization, frame grabbing, and streaming functionality.
    """

    def __init__(self) -> None:
        """Initialize a new ZEDCamera instance with default parameters."""
        ...

    def open(self) -> bool:
        """
        Open the ZED camera with default initialization parameters.

        Returns:
            True if camera opened successfully, False otherwise.
            Use get_last_error() to get error details on failure.
        """
        ...

    def enable_streaming(self, port: int = 30000) -> bool:
        """
        Enable streaming on the specified port.

        Args:
            port: TCP port number for streaming (default: 30000)

        Returns:
            True if streaming enabled successfully, False otherwise.
            Use get_last_error() to get error details on failure.
        """
        ...

    def grab(self) -> bool:
        """
        Grab a new frame from the camera.

        Returns:
            True if frame grabbed successfully, False otherwise.
            Use get_last_error() to get error details on failure.
        """
        ...

    def disable_streaming(self) -> None:
        """Disable streaming and stop the streaming thread."""
        ...

    def close(self) -> None:
        """Close the camera and release all resources."""
        ...

    def get_frame_count(self) -> int:
        """
        Get the total number of frames grabbed since camera was opened.

        Returns:
            Number of frames successfully grabbed.
        """
        ...

    def get_last_error(self) -> str:
        """
        Get the last error message from ZED SDK operations.

        Returns:
            Error message string, empty if no error occurred.
        """
        ...

# Module version
__version__: str
