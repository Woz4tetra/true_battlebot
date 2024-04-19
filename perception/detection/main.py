from typing import cast
import argparse
from bw_shared.environment import get_robot, get_map

from camera.camera_loader import load_camera
from config.config_loader import load_config

from typing import Protocol


class CommandLineArgs(Protocol):
    config_dir: str

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("config_dir", type=str, required=True)
    args: CommandLineArgs = cast(CommandLineArgs, parser.parse_args())

    config_dir = args.config_dir
    config = load_config(config_dir, get_robot())
    camera = load_camera(config.camera)
    print(camera)


if __name__ == "__main__":
    main()
