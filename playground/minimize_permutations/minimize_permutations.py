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
                    2   7    1.5  1.9
                    3   7    4    4

    optimal result:
    [0, 2, 3]
    The result has one element for each filter encoding which measurement index to assign to the filter
    """
    n = distances.shape[1]
    indices = np.arange(n)
    permutations = np.array(list(itertools.permutations(indices)))
    permutation_distances = np.sum(distances[permutations], axis=1)
    min_index = np.argmin(permutation_distances)
    return permutations[min_index]


distances = np.array(
    [
        [1, 4, 10],
        [3.5, 2, 5],
        [7, 1.5, 1.9],
        [7, 4, 4],
    ]
)

minimized = minimize_permutation(distances)
print(minimized)
