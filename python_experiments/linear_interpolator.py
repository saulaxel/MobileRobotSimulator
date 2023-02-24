from __future__ import division
class LinearInterpolator:
    def __init__(self, x1, x2, y1, y2):
        """
        Creates a LinearInterpolator
        """

        self.x1, self.x2, self.y1, self.y2 = x1, x2, y1, y2
        self.conversion_rate = (y2 - y1) / (x2 - x1)


    def convert(self, x):
        return self.y1 + (x - self.x1) * self.conversion_rate

    def inverse_convert(self, y):
        return self.x1 + (y - self.y1) / self.conversion_rate


if __name__ == '__main__':
    from other_math_wrappers import isclose
    li_scaled = LinearInterpolator(0, 1, 0, 2)

    assert isclose(li_scaled.convert(.5), 1)
    assert isclose(li_scaled.inverse_convert(1), .5)

    li_flipped = LinearInterpolator(0, 1, 1, 0)

    assert isclose(li_flipped.convert(0), 1)
    assert isclose(li_flipped.convert(.5), .5)
    assert isclose(li_flipped.inverse_convert(1), 0)

    li_scaled_flipped = LinearInterpolator(0, 1, 2, 0)

    assert isclose(li_scaled_flipped.convert(0), 2)
    assert isclose(li_scaled_flipped.convert(.5), 1)
    assert isclose(li_scaled_flipped.convert(1), 0)

    li_scaled_flipped2 = LinearInterpolator(0, 2, 1, 0)

    assert isclose(li_scaled_flipped2.convert(0), 1)
    assert isclose(li_scaled_flipped2.convert(.5), .75)
    assert isclose(li_scaled_flipped2.convert(1), .5)
    assert isclose(li_scaled_flipped2.convert(2), 0)
