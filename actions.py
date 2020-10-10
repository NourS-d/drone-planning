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
