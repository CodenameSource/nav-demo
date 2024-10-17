from Smav import Waypoint

class WaypointManager:
    def __init__(self):
        self.waypoints = []
        self.stored_waypoints = []

    def add_waypoint(self, waypoint):
        self.waypoints.append(waypoint)

    def squash_waypoints(self):
        tmp_waypoints = self.waypoints.copy()

        prvs_waypoint = tmp_waypoints[0]

        for waypoint in tmp_waypoints[1:]:
            if waypoint[1] == prvs_waypoint[1]:
                prvs_waypoint[0].lat = waypoint[0].lat
                prvs_waypoint[0].lon = waypoint[0].lon
                tmp_waypoints.remove(waypoint)
            prvs_waypoint = waypoint
        self.waypoints = tmp_waypoints

    def stash_waypoints(self):
        self.stored_waypoints.extend(self.waypoints)
        self.waypoints = []

    def write_waypoints(self, path):
        with open(path, 'w') as file:
            for waypoint in self.stored_waypoints:
                file.write(f"{waypoint[0].lat},{waypoint[0].lon},{waypoint[0].alt},{waypoint[0].hdg},{waypoint[1]}\n")

    def read_waypoints(self, path):
        with open(path, 'r') as file:
            for line in file:
                try:
                    lat, lon, alt, hdg, direction = line.split(',')
                    self.waypoints.append([Waypoint(float(lat), float(lon), float(alt), float(hdg)), direction])
                except ValueError:
                    continue
