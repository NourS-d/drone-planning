from drone_control import DroneControl
from motion_planner import MotionPlanner
import matplotlib.pyplot as plt
import numpy as np
from udacidrone.connection import MavlinkConnection
from udacidrone.frame_utils import global_to_local
from utils import load_grid_from_csv, get_random_point_from_grid
import time

FLIGHT_ALTITUDE = 6

if __name__ == "__main__":
    # Create coneection to Udacity's Simulator
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True)
    drone = DroneControl(conn)
    drone.start()

    drone.takeoff_altitude = FLIGHT_ALTITUDE

    # Setup grid, start and goal
    grid, n_offset, e_offset = load_grid_from_csv("colliders.csv", FLIGHT_ALTITUDE)
    start_grid = (int(drone.local_position[0] - n_offset),
                  int(drone.local_position[1] - e_offset))
    goal_grid = get_random_point_from_grid(grid)

    # Plot grid and goal
    plt.matshow(grid, origin='lower')
    plt.plot(start_grid[1], start_grid[0], 'go')
    plt.plot(goal_grid[1], goal_grid[0], 'rx')
    plt.show()

    # Path planner
    print(f"Start: {start_grid}, Goal: {goal_grid}")

    mp = MotionPlanner(grid)
    success, path, cost = mp.run_planner(start_grid, goal_grid)
    if success:
        pruned_path = np.array(mp.prune_path(path))
        plt.matshow(grid, origin='lower')
        plt.plot(pruned_path[:, 1], pruned_path[:, 0], 'b')
        plt.show()
        waypoints = [[p[0] + n_offset, p[1] + e_offset,
                      FLIGHT_ALTITUDE, 0] for p in pruned_path]
        print(waypoints)
        drone.waypoints = waypoints

        # Draw position
        plt.matshow(grid, origin='lower')
        plt.plot(pruned_path[:, 1], pruned_path[:, 0], 'b')
        while drone.in_mission:
            position = (int(drone.local_position[0] - n_offset),
                          int(drone.local_position[1] - e_offset))

            plt.scatter(position[1], position[0],
                        5, "#14d9b1")
            plt.pause(0.05)
            pass
        plt.show()
