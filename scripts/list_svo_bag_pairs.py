#!/usr/bin/env python
from bw_shared.script_tools.directories import BAGS_DIR, SVO_DIR
from bw_shared.script_tools.list_files import list_files


def main() -> None:
    bags = list_files(BAGS_DIR, "bag", extension_in_key=False)
    svos = list_files(SVO_DIR, "svo2", extension_in_key=False)
    bags.pop("")
    svos.pop("")

    bag_names = set(bags.keys())
    svo_names = set(svos.keys())

    intersection_names = bag_names.intersection(svo_names)
    bag_only_names = bag_names.difference(svo_names)
    svo_only_names = svo_names.difference(bag_names)

    for name in bag_only_names:
        print(f"Bag only: {bags[name]}")

    for name in svo_only_names:
        print(f"SVO only: {svos[name]}")

    for name in intersection_names:
        print(f"{name} -> {bags[name]} {svos[name]}")

    if len(intersection_names) == 0:
        print("No matching pairs found")


if __name__ == "__main__":
    main()
