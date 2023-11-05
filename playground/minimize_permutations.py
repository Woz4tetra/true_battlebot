import itertools
from typing import List, Tuple

import numpy as np

cached_permutations = {}


def minimize_permutation(distances: np.ndarray) -> List[Tuple[int, int]]:
    """
    this function minimizes permutations of distances
    example:
        distances   filter id
                        0    1    2
    measurements id
                    0   1    4    10
                    1   3.5  2    5
                    2   7    1.5  4
                    3   7    4    1.9

    optimal result:
    [0, 2, 3]

    Sums to 4.9

    The result has one element for each filter encoding which measurement index to assign to the filter
    """
    num_measurements = distances.shape[0]
    num_filters = distances.shape[1]
    should_flip = num_measurements < num_filters
    if should_flip:
        num_measurements, num_filters = num_filters, num_measurements
        distances = distances.T
    indices = np.arange(num_measurements)
    if num_measurements not in cached_permutations:
        cached_permutations[num_measurements] = np.array(list(itertools.permutations(indices, num_filters)))
    permutations = cached_permutations[num_measurements]
    permutation_distances = np.sum(distances[permutations, tuple(np.arange(num_filters))], axis=1)
    min_index = np.argmin(permutation_distances)
    mapping = []
    for matched_column, matched_row in enumerate(permutations[min_index]):
        if should_flip:
            mapping.append((matched_row, matched_column))
        else:
            mapping.append((matched_column, matched_row))
    return mapping


distances = np.array(
    [
        [1, 4, 10],
        [3.5, 5, 2],
    ]
)

minimized = minimize_permutation(distances)
print("minimized", minimized)
for filter_index, measurement_index in minimized:
    print(filter_index, measurement_index, end=": ")
    print(distances[measurement_index, filter_index])
print()

distances = np.array(
    [
        [1, 4, 10],
        [3.5, 2, 5],
        [7, 1.5, 4],
        [7, 4, 1.9],
    ]
)

minimized = minimize_permutation(distances)
print(minimized)
for filter_index, measurement_index in minimized:
    print(filter_index, measurement_index, end=": ")
    print(distances[measurement_index, filter_index])
