import argparse
from typing import cast

import argcomplete
from perception_tools.directories.config_directory import ConfigType, list_configs

from auto_label.command_line_args import CommandLineArgs


def parse_args() -> CommandLineArgs:
    parser = argparse.ArgumentParser()

    config_names = [config.stem for config in list_configs(ConfigType.AUTO_LABEL)]
    parser.add_argument("config", type=str, default="default", choices=config_names, nargs="?")

    argcomplete.autocomplete(parser)

    args = parser.parse_args()
    return cast(CommandLineArgs, args)


def main() -> None:
    args = parse_args()

    from auto_label.ui.app import App

    App(args).run()


if __name__ == "__main__":
    main()
