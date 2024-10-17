from WaypointManager import WaypointManager, Waypoint

def load_scenario(scenario_path, WaypointManager):
    with open(scenario_path, 'r') as file:
        for line in file:
            try:
                lat, lon, alt, hdg = line.split(',')
                WaypointManager.add_waypoint(Waypoint(float(lat), float(lon), float(alt), float(hdg)))
            except ValueError:
                continue

class Scenario:
    def __init__(self, start_point, target_point, scenario_path=None):
        self.waypoint_manager = WaypointManager()

        self.scenario_path = scenario_path
        if scenario_path is not None:
            load_scenario(scenario_path, self.waypoint_manager)
        else:
