import argparse
import os
from glob import glob
from typing import cast

import argcomplete
from app.field_label.command_line_args import CommandLineArgs

BAGS_DIR = "/data/bags"
VIDEOS_DIR = "/data/videos"
SVO_DIR = "/data/svo"
CONFIGS_DIR = os.path.join(os.path.dirname(__file__), "..", "configs", "field_label_tool")


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
    bag_files = find_files(BAGS_DIR, "*.bag")
    video_files = find_files(VIDEOS_DIR, "*.mp4", "*.avi")
    config_files = find_files(CONFIGS_DIR, "*.toml")
    svo_files = find_files(SVO_DIR, "*.svo2")

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        choices=config_files.keys(),
        default="",
    )

    subparsers = parser.add_subparsers(dest="command")

    bag_parser = subparsers.add_parser("bag")
    bag_parser.add_argument(
        "bag_file",
        type=str,
        choices=bag_files.keys(),
    )

    subparsers.add_parser("topic")
    nhrl_video_parser = subparsers.add_parser("nhrl")
    nhrl_video_parser.add_argument(
        "svo_file",
        type=str,
        choices=svo_files.keys(),
    )
    nhrl_video_parser.add_argument(
        "video_file",
        type=str,
        choices=video_files.keys(),
    )

    svo_parser = subparsers.add_parser("svo")
    svo_parser.add_argument(
        "svo_file",
        type=str,
        choices=svo_files.keys(),
    )

    argcomplete.autocomplete(parser)

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        raise RuntimeError("No command specified")

    match args.command:
        case "bag":
            args.bag_file = bag_files[args.bag_file]
        case "svo":
            args.svo_file = svo_files[args.svo_file]
        case "nhrl":
            args.svo_file = svo_files[args.svo_file]
            args.video_file = video_files[args.video_file]

    if not args.config:
        args.config = config_files[args.command + ".toml"]
    else:
        args.config = config_files[args.config]

    return cast(CommandLineArgs, args)


def main() -> None:
    args = parse_args()

    from app.field_label.field_label_app_runner import run_app

    run_app(args)


if __name__ == "__main__":
    main()
