import numpy as np


def load_grid_from_csv(csv, drone_alt=10, inflation=3):
    """Returns the grid representation of the given obstacle data based on the
    drone altitude and inflation."""
    data = np.loadtxt(csv, delimiter=',', dtype='Float64', skiprows=2)

    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    north_size = int(np.ceil((north_max - north_min + 1)))
    east_size = int(np.ceil((east_max - east_min + 1)))

    grid = np.zeros((north_size, east_size))

    for row in data:
        north, east, alt, d_north, d_east, d_alt = row

        # If inflated obstacle is higher than the drone flying altitude,
        # mark this position as an obstacle.
        if alt + d_alt + inflation > drone_alt:
            obstacle = [
                int(np.clip(north - d_north - inflation -
                            north_min, 0, north_size-1)),
                int(np.clip(north + d_north + inflation -
                            north_min, 0, north_size-1)),
                int(np.clip(east - d_east - inflation -
                            east_min, 0, east_size-1)),
                int(np.clip(east + d_east + inflation -
                            east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


def get_random_point_from_grid(grid):
    """Returns a random non-obstacle point on a 2D grid."""

    n, m = grid.shape
    x, y = np.random.randint(0, n), np.random.randint(0, m)

    while grid[x, y] == 1:
        x, y = np.random.randint(0, n), np.random.randint(0, m)

    return (x, y)
