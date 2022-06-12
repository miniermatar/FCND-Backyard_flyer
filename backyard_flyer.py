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
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            # coordinate conversion
            altitude = -1.0 * self.local_position[2]
            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                time.sleep(1)
                self.calculate_box()
        if self.flight_state == States.WAYPOINT:
            if abs(self.local_position[0]-self.target_pos[0])<0.15 and abs(self.local_position[1]-self.target_pos[1])<0.15:
                self.waypoint_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.2) and
            abs(self.local_position[2]) < 0.04):
                self.disarming_transition()

    def state_callback(self):
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
        self.all_waypoints=[[-15,0,3],[-15,15,3],[0,15,3],[0,0,3]]
        self.waypoint_transition()#North, East Altitute


    def arming_transition(self):
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],self.global_position[1],self.global_position[2])
        self.flight_state = States.ARMING
        print("arming transition")

    def takeoff_transition(self):
        target_altitute=3.0
        self.target_position[2]=target_altitute
        self.takeoff(target_altitute)
        self.flight_state = States.TAKEOFF
        print("takeoff transition")

    def waypoint_transition(self):
        if self.all_waypoints:
            self.target_pos = self.all_waypoints.pop(0)
            self.cmd_position(self.target_pos[0], self.target_pos[1], self.target_pos[2], 0)  # heading in radians=0
            self.flight_state = States.WAYPOINT
            print("waypoint transition: {}".format(self.target_pos))
        else:
            self.landing_transition()

    def landing_transition(self):
        self.land()
        self.flight_state = States.LANDING
        print("landing transition")

    def disarming_transition(self):
        self.disarm()
        self.flight_state = States.DISARMING
        print("disarm transition")

    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
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
