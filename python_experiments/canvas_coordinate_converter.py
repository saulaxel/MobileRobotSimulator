from linear_interpolator import LinearInterpolator


class CanvasCoordinateConverter:
    def __init__(self, x_min, x_max, y_min, y_max,
                 canvas_width, canvas_height):
        self.x_converter = LinearInterpolator(x_min, x_max, 0, canvas_width)
        # Deals with the inverted y-axis by switching order of canvas_height
        # and 0
        self.y_converter = LinearInterpolator(y_min, y_max, canvas_height, 0)


    def to_canvas(self, x, y):
        return (self.x_converter.convert(x),
                self.y_converter.convert(y))


    def to_cartesian(self, cx, cy):
        return (self.x_converter.inverse_convert(cx),
                self.y_converter.inverse_convert(cy))


if __name__ == '__main__':
    from other_math_wrappers import isclose
    x_min = -10
    x_max = 10
    y_min = -5
    y_max = 5
    canvas_size = 300

    coor_converter = CanvasCoordinateConverter(x_min, x_max, y_min, y_max,
                                               canvas_width=canvas_size,
                                               canvas_height=canvas_size)

    cx, cy = coor_converter.to_canvas(0, 0)
    assert isclose(cx, 150) and isclose(cy, 150)

    cx0, cy0 = coor_converter.to_canvas(-10, -5)
    assert isclose(cx0, 0) and isclose(cy0, 300)

    cx1, cy1 = coor_converter.to_canvas(-10, 5)
    assert isclose(cx1, 0) and isclose(cy1, 0)

    cx2, cy2 = coor_converter.to_canvas(10, -5)
    assert isclose(cx2, 300) and isclose(cy2, 300)

    cx3, cy3 = coor_converter.to_canvas(10, 5)
    assert isclose(cx3, 300) and isclose(cy3, 0)

    x, y = coor_converter.to_cartesian(150, 150)
    assert isclose(x, 0) and isclose(y, 0)

    x0, y0 = coor_converter.to_cartesian(0, 300)
    assert isclose(x0, -10) and isclose(y0, -5)

    x1, y1 = coor_converter.to_cartesian(0, 0)
    assert isclose(x1, -10) and isclose(y1, 5)

    x2, y2 = coor_converter.to_cartesian(300, 300)
    assert isclose(x2, 10) and isclose(y2, -5)

    x3, y3 = coor_converter.to_cartesian(300, 0)
    assert isclose(x3, 10) and isclose(y3, 5)

