from drone_control import DroneControl
from udacidrone.connection import MavlinkConnection


if __name__ == "__main__":
    # Create coneection to Udacity's Simulator
    conn = MavlinkConnection('tcp:127.0.0.1:5760')

    drone = DroneControl(conn)
    drone.start()
