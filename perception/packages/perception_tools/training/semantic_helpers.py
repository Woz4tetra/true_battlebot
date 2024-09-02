import csv

from bw_shared.enums.label import ModelLabel


def write_classes(classes: dict[int, ModelLabel], path: str) -> None:
    with open(path, mode="w") as file:
        writer = csv.writer(file)
        writer.writerow(["Pixel Value", "Class"])
        for index, class_name in classes.items():
            writer.writerow([index, class_name.value])


def read_classes(path: str) -> dict[int, ModelLabel]:
    with open(path, mode="r") as file:
        reader = csv.reader(file)
        next(reader)  # Skip header
        return {int(row[0]): ModelLabel(row[1]) for row in reader}
