import argparse
import os
from glob import glob
from typing import cast

import argcomplete
from app.field_label.command_line_args import CommandLineArgs

BAGS_DIR = "/media/storage/bags"


def parse_args() -> CommandLineArgs:
    search_dir = os.path.abspath(BAGS_DIR)
    paths = [f for f in glob(os.path.join(search_dir, "**", "*.bag"), recursive=True)]
    files = {f.replace(search_dir + "/", ""): f for f in paths}
    files[""] = ""

    parser = argparse.ArgumentParser()

    subparsers = parser.add_subparsers(dest="command")

    bag_parser = subparsers.add_parser("bag")
    bag_parser.add_argument(
        "bag_file",
        type=str,
        choices=files.keys(),
    )
    bag_parser.add_argument(
        "cloud_topic",
        type=str,
        default="/camera_0/point_cloud/cloud_registered",
        nargs="?",
    )

    topic_parser = subparsers.add_parser("topic")

    argcomplete.autocomplete(parser)

    args = parser.parse_args()

    match args.command:
        case "bag":
            args.bag_file = files[args.bag_file]

    return cast(CommandLineArgs, args)


def main() -> None:
    args = parse_args()

    from app.field_label.field_label_app import FieldLabelApp

    FieldLabelApp(args).run()


if __name__ == "__main__":
    main()
