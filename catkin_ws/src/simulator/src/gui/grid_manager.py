

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

    gm.grid_and_down(element11)
    gm.grid_and_down(element12)
    gm.grid_and_down(element13)
    gm.grid_and_down(element14)

    gm.return_base()
    gm.right()

    gm.grid_and_down(element21)
    gm.grid_and_down(element22)
    gm.grid_and_down(element23)
    gm.grid_and_down(element24)
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


    def down(self, desp=1):
        self.curr_row += desp

    def up(self, desp=1):
        self.curr_row -= desp

    def right(self, desp=1):
        self.curr_col += desp

    def left(self, desp=1):
        self.curr_col -= desp

    def return_base(self):
        self.curr_col = self.base_col
        self.curr_row = self.base_row


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


    def grid_and_down(self, element, sticky = None, padx = None, pady = None,
                      **kwargs):
        self.grid(element, sticky, padx, pady, **kwargs)
        self.down()


    def grid_and_right(self, element, sticky = None, padx = None, pady = None,
                       **kwargs):
        self.grid(element, sticky, padx, pady, **kwargs)
        self.right()


if __name__ == '__main__':
    # Creating a mock class with the grid method
    class TestGrid:
        def __init__(self):
            self.column = 0
            self.row = 0

        def grid(self, column, row, **kwargs):
            self.column = column
            self.row = row


    # With a mock object of the mock class, we test the behaviour of
    # GridManager
    tg = TestGrid()
    gm = GridManager(base_col = 11, base_row = 11)

    gm.grid_and_down(tg)
    assert tg.column == 11 and tg.row == 11
    gm.grid_and_down(tg)
    assert tg.column == 11 and tg.row == 12
    gm.grid_and_down(tg)
    assert tg.column == 11 and tg.row == 13
    gm.grid_and_down(tg)
    assert tg.column == 11 and tg.row == 14

    gm.return_base()
    gm.right()

    gm.grid_and_down(tg)
    assert tg.column == 12 and tg.row == 11
    gm.grid_and_down(tg)
    assert tg.column == 12 and tg.row == 12
    gm.grid_and_down(tg)
    assert tg.column == 12 and tg.row == 13
    gm.grid_and_down(tg)
    assert tg.column == 12 and tg.row == 14
