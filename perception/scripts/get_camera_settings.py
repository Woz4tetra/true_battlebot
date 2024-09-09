import pyzed.sl as sl
import toml
from app.camera.zed.video_settings import Zed2iVideoSettings


def main() -> None:
    camera = sl.Camera()
    if camera.open() != sl.ERROR_CODE.SUCCESS:
        raise Exception("Failed to open ZED camera")
    try:
        settings = Zed2iVideoSettings.from_camera(camera)
        print(toml.dumps(settings.to_dict()))
    finally:
        camera.close()


if __name__ == "__main__":
    main()
