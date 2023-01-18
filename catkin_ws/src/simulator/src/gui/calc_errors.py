import numpy as np

def calculate_errors(data):
    """
    Calculates:
        total_err -> sum of (yi - y'i)^2
        va        -> total_err / (n - 2)

    Args:
        data -> Array of shape n*2

        0 [y0, y'0]
        1 [y1, y'1]
        ...
        ...
        ...
        n [yn, y'n]
    """
    n = data.shape[0]
    diff = data[:, 0] - data[:, 1]

    mean = sum(np.abs(diff)) / n
    var = sum(diff * diff) / (n - 2)

    return mean, var


if __name__ == '__main__':
    from math import isclose
    data = np.array([
        [1, 1.1],
        [2, 2.1],
        [3, 3.1],
        [4, 4.1]
    ])
    mean, var = calculate_errors(data)
    print(mean)
    assert isclose(mean, .1)
    assert isclose(var, .02)
