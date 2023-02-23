import numpy as np
import aux_stats
import tkinter as tk

canvas_size = 300

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
    from math import isclose
    li = LinearInterpolator(0, 1, 0, 2)

    assert isclose(li.convert(.5), 1)
    assert isclose(li.inverse_convert(1), .5)

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





startX = 0.1905
startY = 2.3876

def circ(c, x, y, color="#476042"):
    x1, y1 = x - 1, y - 1
    x2, y2 = x + 1, y + 1
    w.create_oval(x1, y1, x2, y2, fill=color)


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
    from math import isclose
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


#######################################################33
def plot_in_canvas_xy(canvas, functions, colors, diff=None):
    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')

    # X positions
    for f in functions[0::2]:
        print('X', f)
        min_x = min(min_x, np.min(f))
        max_x = max(max_x, np.max(f))

    # Y positions
    for f in functions[1::2]:
        print('Y', f)
        min_y = min(min_y, np.min(f))
        max_y = max(max_y, np.max(f))

    print('Min x', min_x)
    print('Max x', max_x)
    print('Min y', min_y)
    print('Max y', max_y)

    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    # Ensure that the axis are always shown
    if min_x > 0:
        min_x = 0

    if min_y > 0:
        min_y = 0

    if max_x < 0:
        max_x = 0

    if max_y < 0:
        max_y = 0

    # Managing scale
    if diff is None:
        diff = max(max_x - min_x, max_y - min_y)
    else:
        # Already receive diff, Recalculate min and max
        max_x = center_x + diff / 2
        min_x = center_x - diff / 2

        max_y = center_y + diff / 2
        min_y = center_y - diff / 2

    pad_percent = 10.0
    pad = diff * pad_percent / 100.0

    # Set a min pad value
    if pad < 0.01:
        pad = 0.01

    # Add pad_percent % pad to the axis with the largest values and the
    # necessary pad to the other axis so that both of them have the same
    # scale
    if (max_x - min_x) > (max_y - min_y):
        min_x -= pad
        max_x += pad

        ypad = ((max_x - min_x) - (max_y - min_y)) / 2
        min_y -= ypad
        max_y += ypad
    else:
        min_y -= pad
        max_y += pad

        xpad = ((max_y - min_y) - (max_x - min_x)) / 2
        min_x -= xpad
        max_x += xpad

    diff_y = max_y - min_y
    diff_x = max_x - min_x

    # Reference Axis
    x_axis = canvas_size - (0 - min_y) * canvas_size / (max_y - min_y)
    canvas.create_line(0, x_axis, canvas_size, x_axis)
    y_axis = (0 - min_x) * canvas_size / (max_x - min_x)
    canvas.create_line(y_axis, 0, y_axis, canvas_size)

    length = len(functions[0])
    x0_canvas = (startX - min_x) * canvas_size / diff_x
    y0_canvas = canvas_size - (startY - min_y) * canvas_size / diff_y

    for fx, fy, color in zip(functions[0::2], functions[1::2], colors):
        # Values
        for x, y, in zip(fx, fy):
            x1_canvas = (x - min_x) * canvas_size / diff_x
            y1_canvas = canvas_size - (y - min_y) * canvas_size / diff_y

            canvas.create_line(x0_canvas, y0_canvas, x1_canvas, y1_canvas,
                                fill=color)

    return diff

#######################################################

def debug_temp_print(*args, **kwargs):
    print(*args, **kwargs)

top = tk.Tk()

top.geometry(f'{canvas_size}x{canvas_size}')

c = tk.Canvas(top, bg='white', height=f"{canvas_size}")

c.pack()


x = 0

if x == 0:
    data = np.loadtxt('./test_advance.dat', delimiter=' ', dtype=float)
elif x == 1:
    pass

diff = plot_in_canvas_xy(canvas=c,
                         colors=('red', 'blue'),
                         functions=(data[:, 0], data[:, 1],
                                    data[:, 2], data[:, 3]))


#######################################################

top.mainloop()
