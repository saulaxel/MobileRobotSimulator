import numpy as np
import aux_stats
import tkinter as tk
from other_math_wrappers import isclose

from canvas_coordinate_converter import CanvasCoordinateConverter

canvas_size = 300

startX = 0.1905
startY = 2.3876

def circ(canvas, cx, cy, color="#476042"):
    cx1, cy1 = cx - 2, cy - 2
    cx2, cy2 = cx + 2, cy + 2
    canvas.create_oval(cx1, cy1, cx2, cy2, fill=color)


def vertical_line(canvas, canvas_height, cx):
    canvas.create_line(cx, 0, cx, canvas_height)

def horizontal_line(canvas, canvas_width, cy):
    canvas.create_line(0, cy, canvas_width, cy)


def include_zero_in_range(min_val, max_val):
    if min_val > 0:
        min_val = 0

    if max_val < 0:
        max_val = 0

    return min_val, max_val

#######################################################33
def get_extreme_values(functions):
    """
    Gets the min and max value inside a collection of functions
    Each function is represented as an 1D array of values
    """
    # Min value stars infinity so any number is less than the initial
    # Max value starts as -infinity for the same reason
    min_val, max_val = float('inf'), float('-inf')

    for f in functions:
        min_val = min(min_val, min(f))
        max_val = max(max_val, max(f))

    return min_val, max_val

def plot_in_canvas_xy(functions, function_colors,
                      canvas, canvas_size,
                      diff=None, min_pad=1e-6,
                      always_show_axes=False):
    """
    Plots several functions received as a collection of 1D arrays using the
    colors provided with the argument function_colors in a square tkinter
    canvas.

    The function is intended to plot both axes using the same scale. The *diff*
    argument establishes the minimum required size for the axes
    """

    x_functions = functions[0::2]
    y_functions = functions[1::2]

    # X extremes
    min_x, max_x = get_extreme_values(x_functions)
    min_x = min(min_x, startX) # Also take into account the starting point
    max_x = max(max_x, startX)
    # Y extremes
    min_y, max_y = get_extreme_values(y_functions)
    min_y = min(min_y, startY)
    max_y = max(max_y, startY)


    debug_temp_print('Min x', min_x)
    debug_temp_print('Max x', max_x)
    debug_temp_print('Min y', min_y)
    debug_temp_print('Max y', max_y)

    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    debug_temp_print('center_x', center_x)
    debug_temp_print('center_y', center_y)

    if always_show_axes:
        # Include 0 in the range to be shown
        min_x, max_x = include_zero_in_range(min_x, max_x)
        min_y, max_y = include_zero_in_range(min_y, max_y)

    # Managing scale so that X and Y have the same scaling (provided that we
    # continue to use a square canvas or else things have to be further modified)
    original_diffX = max_x - min_x
    original_diffY = max_y - min_y
    if diff is None:
        debug_temp_print('diffX', original_diffX)
        debug_temp_print('diffY', original_diffY)
        diff = max(original_diffX, original_diffY)
        debug_temp_print('diff', diff)
    else:
        # Already receive diff, Recalculate min and max
        max_x = center_x + diff / 2
        min_x = center_x - diff / 2

        max_y = center_y + diff / 2
        min_y = center_y - diff / 2


    # Add (pad_percent x pad) to the axis with the largest values and the
    # necessary pad to the other axis so that both of them have the same
    # scale
    pad_percent = 10.0
    pad = diff * pad_percent / 100.0
    debug_temp_print('pad', pad)

    if pad < min_pad:
        pad = min_pad

    debug_temp_print('pad', pad)
    if original_diffX > original_diffY:
        padded_min_x = min_x - pad
        padded_max_x = max_x + pad

        ypad = ((padded_max_x - padded_min_x) - original_diffY) / 2
        padded_min_y = min_y - ypad
        padded_max_y = max_y + ypad
    else:
        padded_min_y = min_y - pad
        padded_max_y = max_y + pad

        xpad = ((padded_max_y - padded_min_y) - original_diffX) / 2
        padded_min_x = min_x - xpad
        padded_max_x = max_x + xpad

    debug_temp_print('padded_min_x', padded_min_x)
    debug_temp_print('padded_max_x', padded_max_x)
    debug_temp_print('padded_min_y', padded_min_y)
    debug_temp_print('padded_max_y', padded_max_y)

    padded_diff = padded_max_y - padded_min_y
    assert isclose((padded_max_x - padded_min_x), (padded_max_y - padded_min_y))
    debug_temp_print('padded_diff', padded_diff)

    # Ploting

    # As noted earlier, in canvas of different height and width the scaling
    # formulas would have to be modified before creating following object if we
    # want the same scale in both axes.
    ccc = CanvasCoordinateConverter(x_min=padded_min_x, x_max=padded_max_x,
                                    y_min=padded_min_y, y_max=padded_max_y,
                                    canvas_width=canvas_size,
                                    canvas_height=canvas_size)

    debug_temp_print('c_min_x', ccc.x_to_canvas(min_x),
                     'c_max_x', ccc.x_to_canvas(max_x))

    debug_temp_print('c_min_y', ccc.y_to_canvas(min_y),
                     'c_max_y', ccc.y_to_canvas(max_y))


    # Drawing reference axes
    cx_axis, cy_axis = ccc.to_canvas(0, 0)
    vertical_line(canvas, canvas_height=canvas_size, cx=cx_axis)
    horizontal_line(canvas, canvas_width=canvas_size, cy=cy_axis)

    # The plots will be from the starting point of movement (startX, startY) to
    # the final point (x, y)
    cx0, cy0 = ccc.to_canvas(startX, startY)
    circ(canvas, cx0, cy0, color='red')

    for fx, fy, color in zip(x_functions, y_functions, function_colors):
        # Values
        for x, y, in zip(fx, fy):
            cx1, cy1 = ccc.to_canvas(x, y)

            circ(canvas, cx1, cy1, color=color)

            canvas.create_line(cx0, cy0, cx1, cy1,
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

diff = plot_in_canvas_xy(function_colors=('red', 'blue'),
                         functions=(data[:, 0], data[:, 1],
                                    data[:, 2], data[:, 3]),
                         canvas=c,
                         canvas_size=canvas_size,
                         always_show_axes=False)


#######################################################

top.mainloop()



if __name__ == '__main__':
    pass
