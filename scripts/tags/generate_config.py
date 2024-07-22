#!/usr/bin/env python
# PYTHON_ARGCOMPLETE_OK
import argparse
import glob

import argcomplete
from tag_tools import generate_from_config


def main() -> None:
    parser = argparse.ArgumentParser(description="config_tag")
    parser.add_argument("config", choices=list(glob.glob("*.toml")), help="path to config")
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    config = args.config
    generate_from_config(config)


if __name__ == "__main__":
    main()
