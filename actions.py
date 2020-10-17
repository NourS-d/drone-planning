from enum import Enum
from numpy import sqrt


class Actions(Enum):

    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    EAST = (0, 1, 1)
    WEST = (0, -1, 1)
    NORTH_WEST = (-1, -1, sqrt(2))
    NORTH_EAST = (-1, 1, sqrt(2))
    SOUTH_WEST = (1, -1, sqrt(2))
    SOUTH_EAST = (1, 1, sqrt(2))

    @property
    def delta(self):
        return (self.value[0], self.value[1])

    @property
    def cost(self):
        return self.value[2]

    @staticmethod
    def valid_grid_actions(grid, position):
        """Returns a list a valid actions in a grid at a certain position."""
        valid_actions = []
        n, m = grid.shape
        x, y = position

        if x - 1 > 0:
            if grid[x-1, y] == 0:
                valid_actions.append(Actions.NORTH)
            if y - 1 > 0 and grid[x-1, y-1] == 0:
                valid_actions.append(Actions.NORTH_WEST)
            if y + 1 < m and grid[x-1, y+1] == 0:
                valid_actions.append(Actions.NORTH_EAST)
        if x + 1 < n:
            if grid[x+1, y] == 0:
                valid_actions.append(Actions.SOUTH)
            if y - 1 > 0 and grid[x+1, y-1] == 0:
                valid_actions.append(Actions.SOUTH_WEST)
            if y + 1 < m and grid[x+1, y+1] == 0:
                valid_actions.append(Actions.SOUTH_EAST)

        if y - 1 > 0 and grid[x, y-1] == 0:
            valid_actions.append(Actions.WEST)
        if y + 1 < m and grid[x, y+1] == 0:
            valid_actions.append(Actions.EAST)

        return valid_actions
