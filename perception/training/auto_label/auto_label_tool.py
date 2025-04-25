import argparse
from typing import cast

import argcomplete

from auto_label.command_line_args import CommandLineArgs


def parse_args() -> CommandLineArgs:
    parser = argparse.ArgumentParser()

    argcomplete.autocomplete(parser)

    args = parser.parse_args()
    return cast(CommandLineArgs, args)


def main() -> None:
    args = parse_args()


if __name__ == "__main__":
    main()
