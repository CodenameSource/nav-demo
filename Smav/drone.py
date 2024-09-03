import re

from pymavlink import mavutil

from Smav.control import set_mode, takeoff, land, goto, get_location
from Smav.util import Waypoint

class Drone:
    def __init__(self, host, port=5760, tcp=True):
        if not tcp:
            raise NotImplementedError("Only TCP is supported")

        self.host = host
        self.port = port

        self.master = self.connect()

    def connect(self):
        master = mavutil.mavlink_connection(f'tcp:{self.host}:{self.port}')\

        heartbeat = master.wait_heartbeat(timeout=10)

        if heartbeat is None:
            raise ConnectionError("No heartbeat received")

        return master

    def disconnect(self):
        self.master.close()

    def reconnect(self):
        self.disconnect()
        self.master = self.connect()

    def arm(self):
        self.master.arducopter_arm()
        self.master.motors_armed_wait()

    def set_mode(self, mode: str):
        set_mode(self.master, mode)

    def takeoff(self, alt: int):
        takeoff(self.master, alt)

    def land_bellow(self):
        current_location = self.get_location()
        lat = current_location['lat']
        lon = current_location['lon']
        landing_alt = current_location['alt'] - current_location['relative_alt'] + .05

        land(self.master, lat, lon, landing_alt)

    def goto(self, waypoint: Waypoint):
        goto(self.master, waypoint.lat, waypoint.lon, waypoint.alt)

    def get_location(self):
        return get_location(self.master)