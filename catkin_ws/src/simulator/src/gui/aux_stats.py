import numpy as np

def calculate_statistics(data):
    """
    Calculates:
        mean -> (sum of yi) / n
        var  -> (sum of (yi - mean)) / n

    Args:
        data -> Array of shape n x 1

        0 [y0]
        1 [y1]
        ...
        ...
        ...
        n [yn]
    """
    n = data.shape[0]
    mean = sum(data) / n
    var = sum((data - mean) ** 2) / n

    return mean, var


def calculate_errors(data):
    """
    Calculates:

        error mean -> (sum of (yi - y'i)) / n
        error var  -> (sum of (yi - y'i)^2) / (n - 2)

    Args:
        data -> Array of shape n x 2

        0 [y0, y'0]
        1 [y1, y'1]
        ...
        ...
        ...
        n [yn, y'n]
    """
    n = data.shape[0]
    diff = data[:, 0] - data[:, 1]

    err_mean = sum(np.abs(diff)) / n
    err_var = sum(diff * diff) / (n - 2)

    return err_mean, err_var


if __name__ == '__main__':
    try:
        from math import isclose
    except:
        def isclose(a, b):
            return abs(a - b) < 0.001

    data = np.array([
        [1, 1.1],
        [2, 2.1],
        [3, 3.1],
        [4, 4.1]
    ])

    mean1, var1 = calculate_statistics(data[:, 0])
    mean2, var2 = calculate_statistics(data[:, 1])
    err_mean, err_var = calculate_errors(data)
    assert isclose(mean1, 2.5)
    assert isclose(var1, 1.25)
    assert isclose(mean2, 2.6)
    assert isclose(var2, 1.25)
    assert isclose(err_mean, .1)
    assert isclose(err_var, .02)


