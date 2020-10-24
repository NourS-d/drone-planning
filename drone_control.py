import time
from enum import IntEnum, auto
import numpy as np
from udacidrone import Drone
from udacidrone.messaging import MsgID


class States(IntEnum):
    MANUAL = auto()
    ARMED = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMED = auto()


class DroneControl(Drone):

    def __init__(self, connection, tlog_directory='Logs',
                 tlog_name='TLog.txt'):
        super().__init__(connection,
                         tlog_directory=tlog_directory,
                         tlog_name=tlog_name)

        self.in_mission = True

        # Initial State
        self.state = States.MANUAL

        self.target_position = np.zeros(3)
        self._takeoff_altitude = 3

        self.waypoints = []

        self.register_callback(MsgID.LOCAL_POSITION,
                               self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY,
                               self.velocity_callback)
        self.register_callback(MsgID.STATE,
                               self.state_callback)

    def start(self):
        """Starts the connection to the drone."""

        print("Starting Connection...")
        super().start()

    def stop(self):
        """Closes the connection to the drone."""
        print("Stopping...")
        super().stop()
        self.state = States.MANUAL

    def arm_drone(self):
        """Take over and arm the drone."""

        self.take_control()
        time.sleep(1)  # To make sure we are in guided mode before arming
        self.arm()

        # Set the current global position as the home position
        self.set_home_as_current_position()

        if self.armed:
            self.state = States.ARMED
        else:
            print("Drone can not arm right now. "
                  "Releasing control...")
            #self.release_control()

    def disarm_drone(self):
        """Sends a disarm command to the drone and releases control."""

        self.disarm()
        self.release_control()
        self.state = States.DISARMED

    def takeoff(self):
        print("Taking off!")
        self.state = States.TAKEOFF
        self.target_position[2] = -self.takeoff_altitude  # NED coordinates
        super().takeoff(self.takeoff_altitude)

    def land(self):
        print("Landing drone...")
        super().land()
        self.state = States.LANDING
    
    def start_navigation(self):
        if len(self.waypoints) == 0:
            print("Waiting for waypoints.")
        else:
            print("Starting navigation.")
            self.navigate_to_waypoint()

    def navigate_to_waypoint(self):
        self.state = States.WAYPOINT
        self.target_position = self.waypoints.pop(0)
        print("Target:", self.target_position)
        self.cmd_position(*self.target_position)

    def state_callback(self):
        if self.state == States.MANUAL:
            self.arm_drone()

        elif self.state == States.ARMED:
            self.takeoff()

        elif self.state == States.DISARMED:
            if not (self.armed or self.guided):
                self.stop()

    def local_position_callback(self):
        if self.state == States.TAKEOFF:
            if np.allclose(self.local_position[2],
                           self.target_position[2],
                           rtol=0.05, atol=0):
                self.start_navigation()
        elif self.state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.navigate_to_waypoint()
                else:
                    print("Reached last waypoint.")
                    self.land()

    def velocity_callback(self):
        if self.state == States.LANDING:
            if np.allclose(self.local_velocity, np.zeros(3)) \
               and self.local_position[2] < 0.05:
                self.disarm_drone()

    @property
    def takeoff_altitude(self):
        return self._takeoff_altitude

    @takeoff_altitude.setter
    def takeoff_altitude(self, alt):
        """Set the takeoff altitude of your drone.

        Altitude is currently limited to a 100m.
        """

        if self.state != States.MANUAL:
            print("Set the takeoff altitude when the drone "
                  "is not in operation. Nothing changed!")
            return None

        if alt < 0:
            raise ValueError("Cannot takeoff to a negative altitude.")
        elif alt > 100:
            raise ValueError(
                "Flying too high! See "
                "https://uavcoach.com/drone-laws-in-germany/")
        else:
            self._takeoff_altitude = alt
