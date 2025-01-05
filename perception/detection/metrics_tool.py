import argparse
import os
from glob import glob
from typing import cast

import argcomplete
from app.metrics.command_line_args import CommandLineArgs

VIDEOS_DIR = "/data/videos"
CONFIGS_DIR = os.path.join(os.path.dirname(__file__), "..", "configs", "metrics_tool")


def recursive_glob(directory: str, pattern: str) -> list[str]:
    return [f for f in glob(os.path.join(directory, "**", pattern), recursive=True) if os.path.isfile(f)]


def find_files(directory: str, *patterns: str) -> dict[str, str]:
    search_dir = os.path.abspath(directory)
    paths: list[str] = []
    for pattern in patterns:
        paths += recursive_glob(search_dir, pattern)
    files = {f.replace(search_dir + "/", ""): f for f in paths}
    files[""] = ""
    return files


def parse_args() -> CommandLineArgs:
    video_files = find_files(VIDEOS_DIR, "*.mp4", "*.avi")
    config_files = find_files(CONFIGS_DIR, "*.toml")

    parser = argparse.ArgumentParser()
    parser.add_argument("video_file", type=str, choices=video_files.keys())
    parser.add_argument("config", type=str, choices=config_files.keys())

    argcomplete.autocomplete(parser)

    args = parser.parse_args()

    args.video_file = video_files[args.video_file]
    args.config = config_files[args.config]

    return cast(CommandLineArgs, args)


def main() -> None:
    args = parse_args()

    from app.metrics.metrics_app_runner import run_app

    run_app(args)


if __name__ == "__main__":
    main()
