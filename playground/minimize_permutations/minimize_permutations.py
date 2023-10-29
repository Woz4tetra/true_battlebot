import itertools

import numpy as np


def minimize_permutation(distances: np.ndarray) -> np.ndarray:
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
    n = distances.shape[0]
    m = distances.shape[1]
    indices = np.arange(n)
    permutations = np.array(list(itertools.permutations(indices, m)))
    permutation_distances = np.sum(distances[permutations, tuple(np.arange(m))], axis=1)
    min_index = np.argmin(permutation_distances)
    return permutations[min_index]


distances = np.array(
    [
        [1, 4, 10],
        [3.5, 2, 5],
        [7, 1.5, 4],
        [7, 4, 1.9],
    ]
)

minimized = minimize_permutation(distances)
print(distances[minimized, (0, 1, 2)])
for filter_index, measurement_index in enumerate(minimized):
    print(filter_index, measurement_index)
