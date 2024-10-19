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

    def goto(self, waypoint: Waypoint, hold_time: int = 2, near_threshold_meters: float = 1.5) -> bool:
        """Go to a waypoint.

        Args:
                waypoint (Waypoint): The waypoint to go to.
                hold_time (int, optional): The time to hold at the waypoint. Defaults to 2.
                near_threshold_meters (float, optional): The distance to the waypoint to consider as "near". Defaults to 1.5.
        Returns:
                bool: True if the drone reached the waypoint, False otherwise.
        """
        try:
            return goto(self.master, waypoint.lat, waypoint.lon, waypoint.alt, waypoint.hdg, hold_time, near_threshold_meters)
        except Exception as e:
            print(e)
            return False

    def get_location(self):
        return get_location(self.master)

    @property
    def heading(self):
        return get_location(self.master)['hdg']

    @property
    def location(self) -> dict:
        """Return the current location of the drone.


        Returns:
                dict: A dictionary containing the following keys:

                - lat (float): The latitude of the drone.
                - lon (float): The longitude of the drone.
                - alt (float): The altitude of the drone.
                - relative_alt (float): The relative altitude of the drone.
                - hdg (float): The heading of the drone.
        """
        return get_location(self.master)