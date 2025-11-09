"""Type stubs for recorder_cpp nanobind module"""

from enum import IntEnum

class RESOLUTION(IntEnum):
    HD4K = ...
    QHDPLUS = ...
    HD2K = ...
    HD1536 = ...
    HD1080 = ...
    HD720 = ...
    SVGA = ...
    VGA = ...
    AUTO = ...

class DEPTH_MODE(IntEnum):  # noqa: N801
    NONE = ...
    PERFORMANCE = ...
    QUALITY = ...
    ULTRA = ...
    NEURAL_LIGHT = ...
    NEURAL = ...
    NEURAL_PLUS = ...

class InitParameters:
    """
    Initialization parameters for ZED Camera.
    """

    def __init__(
        self,
        camera_resolution: RESOLUTION = RESOLUTION.AUTO,
        depth_mode: DEPTH_MODE = DEPTH_MODE.NONE,
        sdk_verbose: int = 1,
        camera_fps: int = 30,
    ) -> None:
        """Initialize InitParameters with optional custom settings."""
        ...

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

    def update(self) -> bool:
        """
        Grab a new frame from the camera.

        Returns:
            True if frame grabbed successfully, False otherwise.
            Use get_last_error() to get error details on failure.
        """
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

    def retrieve_image(self) -> "sl.Mat":  # type: ignore  # noqa: F821
        """
        Retrieve the last grabbed image from the camera.

        Returns:
            The last grabbed image as an sl.Mat object.
        """
        ...

# Module version
__version__: str
