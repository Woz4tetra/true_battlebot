import pyzed.sl as sl
from app.camera.zed.svo_camera import SvoCamera
from app.camera.zed.zed_helpers import zed_status_to_str
from perception_tools.messages.camera_data import CameraData


def load_from_svo(svo_path: str, start_time: float = 0.0) -> CameraData:
    camera = SvoCamera(svo_path, "camera_0")
    camera.open()
    camera.set_svo_time(start_time)
    camera_data, status = camera.get_camera_data()
    if status != sl.ERROR_CODE.SUCCESS or camera_data is None:
        raise Exception(f"Error getting camera data: {zed_status_to_str(status)}")
    camera.close()
    camera_data.point_cloud = camera_data.point_cloud.flatten()
    return camera_data
