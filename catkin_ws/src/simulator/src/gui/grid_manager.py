from copy import copy


class GridManager:
    """
    We frequently see this king of usage with tkinter grids:
        element11.grid(column = 11, row = 11, sticky = S, padx = X, pady = Y)
        element12.grid(column = 11, row = 12, sticky = S, padx = X, pady = Y)
        element13.grid(column = 11, row = 13, sticky = S, padx = X, pady = Y)
        element14.grid(column = 11, row = 14, sticky = S, padx = X, pady = Y)

        element21.grid(column = 12, row = 11, sticky = S, padx = X, pady = Y)
        element22.grid(column = 12, row = 12, sticky = S, padx = X, pady = Y)
        element23.grid(column = 12, row = 13, sticky = S, padx = X, pady = Y)
        element24.grid(column = 12, row = 14, sticky = S, padx = X, pady = Y)

    This auxiliary class intents to shorten this repetition a little.
    We can do the same as above using the following

    gm = GridManager(base_col = 11, base_row = 11, sticky = S, padx = X, pady = Y)

    gm.grid(element11).down()
    gm.grid(element12).down()
    gm.grid(element13).down()
    gm.grid(element14)

    gm.return_base().right()

    gm.grid(element21).down()
    gm.grid(element22).down()
    gm.grid(element23).down()
    gm.grid(element24).down()
    """
    def __init__(self, base_col, base_row,
                 sticky = None,
                 padx = None, pady = None):
        self.base_col = self.curr_col = base_col
        self.base_row = self.curr_row = base_row
        self.sticky = sticky
        self.padx = padx
        self.pady = pady


    def set_base(self, base_col, base_row):
        self.base_col = self.curr_col = base_col
        self.base_row = self.curr_row = base_row
        return self


    def down(self, desp=1):
        self.curr_row += desp
        return self

    def up(self, desp=1):
        self.curr_row -= desp
        return self

    def right(self, desp=1):
        self.curr_col += desp
        return self

    def left(self, desp=1):
        self.curr_col -= desp
        return self

    def return_base(self):
        self.curr_col = self.base_col
        self.curr_row = self.base_row
        return self


    def grid(self, element, sticky = None, padx = None, pady = None, **kwargs):
        if sticky is None:
            sticky = self.sticky
        if padx is None:
            padx = self.padx
        if pady is None:
            pady = self.pady

        element.grid(column = self.curr_col, row = self.curr_row,
                     sticky = sticky, padx = padx, pady = pady,
                     **kwargs)
        return self


if __name__ == '__main__':
    # Creating a mock class with the grid method
    class TestGrid:
        def __init__(self):
            self.column = 0
            self.row = 0
            self.kwargs = {}

        def grid(self, column, row, **kwargs):
            self.column = column
            self.row = row
            self.kwargs = kwargs


    # With a mock object of the mock class, we test the behaviour of
    # GridManager
    tg = TestGrid()
    gm1 = GridManager(base_col=0, base_row=0)
    gm1.grid(tg)
    assert tg.column == 0 and tg.row == 0
    gm1.down().grid(tg)
    assert tg.column == 0 and tg.row == 1
    gm1.right().grid(tg)
    assert tg.column == 1 and tg.row == 1
    gm1.up().grid(tg)
    assert tg.column == 1 and tg.row == 0
    gm1.left().grid(tg)
    assert tg.column == 0 and tg.row == 0


    # Also test the example from
    args = {
        'sticky': 'S',
        'padx': 'X',
        'pady': 'Y',
    }
    args_extra = copy(args)
    args_extra['extra'] = 'E'
    gm2 = GridManager(base_col = 11, base_row = 11, **args)

    gm2.grid(tg).down()
    assert tg.column == 11 and tg.row == 11 and tg.kwargs == args
    gm2.grid(tg, extra = 'E').down()
    assert tg.column == 11 and tg.row == 12 and tg.kwargs == args_extra
    gm2.grid(tg).down()
    assert tg.column == 11 and tg.row == 13
    gm2.grid(tg).down()
    assert tg.column == 11 and tg.row == 14

    gm2.return_base()
    gm2.right()

    gm2.grid(tg).down()
    assert tg.column == 12 and tg.row == 11
    gm2.grid(tg).down()
    assert tg.column == 12 and tg.row == 12
    gm2.grid(tg).down()
    assert tg.column == 12 and tg.row == 13
    gm2.grid(tg).down()
    assert tg.column == 12 and tg.row == 14
