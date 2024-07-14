class NavigationError(Exception): ...


class FrameIdMismatchError(NavigationError):
    def __init__(self, expected_frame_id: str, actual_frame_id: str):
        self.expected_frame_id = expected_frame_id
        self.actual_frame_id = actual_frame_id
        self.message = f"Expected frame_id: {expected_frame_id}, but got: {actual_frame_id}"
        super().__init__(self.message)
