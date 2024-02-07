import argparse
import time
from enum import Enum
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.box_size = 10.0
        self.flight_state = States.MANUAL
        self.register_callbacks()

    def register_callbacks(self):
        """Register all callbacks."""
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """Callback for local position updates."""
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            distance = np.linalg.norm(self.target_position[0:2] - self.local_position[0:2])
            if distance < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    self.landing_transition()

    def velocity_callback(self):
        """Callback for velocity updates."""
        if self.flight_state == States.LANDING:
            if abs(self.local_velocity[2]) < 0.1:
                self.disarming_transition()

    def state_callback(self):
        """Callback for state updates."""
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def calculate_box(self):
        print("Setting Home")
        local_waypoints = [[10.0, 0.0, 3.0], [10.0, 10.0, 3.0], [0.0, 10.0, 3.0], [0.0, 0.0, 3.0]]
        return local_waypoints

    def arming_transition(self):
        """Transition to the arming state."""
        self.take_control()
        self.arm()
        self.set_home_as_current_position()
        self.flight_state = States.ARMING
        print("arming transition")

    def takeoff_transition(self):
        """Transition to the takeoff state."""
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF
        print("takeoff transition")

    def waypoint_transition(self):
        """Transition to the waypoint state."""
        print("waypoint transition")
        if not self.all_waypoints:
            print("No more waypoints. Mission complete.")
            self.stop()
            return
        self.target_position = self.all_waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """Transition to the landing state."""
        self.land()
        self.flight_state = States.LANDING
        print("landing transition")

    def disarming_transition(self):
        """Transition to the disarming state."""
        self.disarm()
        self.flight_state = States.DISARMING
        print("disarm transition")

    def manual_transition(self):
        """Transition to the manual state."""
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL
        print("manual transition")

    def start(self):
        """Start the drone mission."""
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
