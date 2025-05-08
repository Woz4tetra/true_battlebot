import argparse
from typing import cast

import argcomplete
from perception_tools.directories.config_directory import ConfigType, list_configs
from perception_tools.directories.data_directory import list_data_directory

from auto_label.command_line_args import CommandLineArgs


def parse_args() -> CommandLineArgs:
    parser = argparse.ArgumentParser()

    config_names = [config.stem for config in list_configs(ConfigType.AUTO_LABEL)]
    keypoint_config_names = {path.stem: path for path in list_data_directory("keypoints_config")}
    parser.add_argument("config", type=str, default="default", choices=config_names, nargs="?")
    parser.add_argument(
        "-k",
        "--keypoints",
        type=str,
        default=next(iter(keypoint_config_names.keys())),
        choices=keypoint_config_names.keys(),
        nargs="?",
        dest="keypoints_config",
    )

    argcomplete.autocomplete(parser)

    args = parser.parse_args()
    args.keypoints_config = keypoint_config_names[args.keypoints_config]
    return cast(CommandLineArgs, args)


def main() -> None:
    args = parse_args()

    from auto_label.ui.app import App

    App(args).run()


if __name__ == "__main__":
    main()
