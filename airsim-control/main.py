import math
from pynput import keyboard
from pynput.keyboard import Key
from time import sleep, time

from inputs import get_gamepad, devices
from pynput.keyboard import KeyCode

from Smav.drone import Drone, Waypoint
from WaypointManager import WaypointManager

host = '127.0.0.1'
port = 5760

DISTANCE = 1

mission_waypoints = []
WayMan = WaypointManager()


def save_mission_waypoints():
    with open("mission_waypoints.txt", "w") as file:
        for waypoint in mission_waypoints:
            file.write(f"{waypoint.lat},{waypoint.lon},{waypoint.alt},{waypoint.hdg}\n")

def quit_on_controller_disconnect():
    if len(devices.gamepads) == 0:
        print("Controller disconnected")
        raise Exception("Controller disconnected")


def drone_connect_and_arm_proc(host, port, arm=True):
    # Initialize the drone with host and port
    drone = Drone(host, port)

    sleep(2)
    drone.reconnect()
    sleep(2)
    drone.reconnect()
    sleep(10)  # >:( I dont know why does 10 seconds work

    drone.set_mode('GUIDED')
    drone.set_mode('GUIDED')
    sleep(10)

    if arm:
        # Arm the drone again and wait for 4 seconds
        drone.arm()

    return drone


def get_new_coordinates(lat, lon, distance, bearing):
    # Convert latitude, longitude, and bearing from degrees to radians
    lat = math.radians(lat)
    lon = math.radians(lon)
    bearing = math.radians(bearing)

    # Earth radius in meters
    R = 6378137

    # Angular distance in radians
    delta = distance / R

    # Calculate new latitude
    lat_new = math.asin(math.sin(lat) * math.cos(delta) +
                        math.cos(lat) * math.sin(delta) * math.cos(bearing))

    # Calculate new longitude
    lon_new = lon + math.atan2(math.sin(bearing) * math.sin(delta) * math.cos(lat),
                               math.cos(delta) - math.sin(lat) * math.sin(lat_new))

    # Convert latitude and longitude back to degrees
    lat_new = math.degrees(lat_new)
    lon_new = math.degrees(lon_new)

    return lat_new, lon_new


def generate_front_waypoint(current_location):
    lat, lon = get_new_coordinates(current_location['lat'], current_location['lon'], DISTANCE, current_location['hdg'])

    return Waypoint(lat, lon, current_location['alt'], current_location['hdg'])


def generate_back_waypoint(current_location):
    lat, lon = get_new_coordinates(current_location['lat'], current_location['lon'], DISTANCE,
                                   current_location['hdg'] + 180)

    return Waypoint(lat, lon, current_location['alt'], current_location['hdg'])


def generate_left_waypoint(current_location):
    lat, lon = get_new_coordinates(current_location['lat'], current_location['lon'], DISTANCE,
                                   current_location['hdg'] - 90)

    return Waypoint(lat, lon, current_location['alt'], current_location['hdg'])


def generate_right_waypoint(current_location):
    lat, lon = get_new_coordinates(current_location['lat'], current_location['lon'], DISTANCE,
                                   current_location['hdg'] + 90)

    return Waypoint(lat, lon, current_location['alt'], current_location['hdg'])


def trace_location(drone, waypoint_manager):
    if len(waypoint_manager.waypoints) == 0:
        return drone.get_location()

    last_waypoint = waypoint_manager.waypoints[-1][0]

    return {"lat": last_waypoint.lat, "lon": last_waypoint.lon, "alt": last_waypoint.alt, "hdg": last_waypoint.hdg}


def main_controller(arm_takeoff=True):
    """Just print out some event infomation when the gamepad is used."""
    drone = drone_connect_and_arm_proc(host, port, arm_takeoff)
    print("Drone connected and armed")
    if arm_takeoff:
        drone.takeoff(5)
        print("Drone took off")

    last_timestamp = 0
    while True:
        quit_on_controller_disconnect()
        events = get_gamepad()
        current_location = trace_location(drone, WayMan)
        for event in events:
            if event.timestamp - last_timestamp < .5:
                continue

            if event.code == "BTN_DPAD_UP" and event.state == 1:
                print("up")
                front_waypoint = generate_front_waypoint(current_location)
                WayMan.add_waypoint([front_waypoint, "front"])

                last_timestamp = event.timestamp
                break
            elif event.code == "BTN_DPAD_DOWN" and event.state == 1:
                print("down")
                back_waypoint = generate_back_waypoint(current_location)
                WayMan.add_waypoint([back_waypoint, "back"])

                last_timestamp = event.timestamp
                break
            elif event.code == "BTN_DPAD_LEFT" and event.state == 1:
                print("left")
                left_waypoint = generate_left_waypoint(current_location)
                WayMan.add_waypoint([left_waypoint, "left"])

                last_timestamp = event.timestamp
                break
            elif event.code == "BTN_DPAD_RIGHT" and event.state == 1:
                print("right")
                right_waypoint = generate_right_waypoint(current_location)
                WayMan.add_waypoint([right_waypoint, "right"])

                last_timestamp = event.timestamp
                break

            elif event.code == "BTN_SOUTH" and event.state == 1:
                print("following on waypoints")
                WayMan.squash_waypoints()

                for waypoint in WayMan.waypoints:
                    drone.goto(waypoint[0], 1)

                WayMan.stash_waypoints()
                break


def main_keyboard(arm_takeoff=True):
    """Just print out some event infomation when the gamepad is used."""
    drone = drone_connect_and_arm_proc(host, port, arm_takeoff)
    print("Drone connected and armed")
    if arm_takeoff:
        drone.takeoff(5)
        print("Drone took off")

    last_key_timestamp = 0
    with keyboard.Events() as events:
        while True:
            # Block at most one second
            event = events.get(1.0)
            if event is None or time() - last_key_timestamp < 1:
                continue
            current_location = trace_location(drone, WayMan)

            if type(event.key) != Key and event.key.char == '/':
                print("Saving current waypoint...")

                current_location = drone.get_location()
                alt = current_location['alt']
                lat = current_location['lat']
                lon = current_location['lon']
                hdg = current_location['hdg']

                current_waypoint = Waypoint(lat, lon, alt, hdg)
                if current_waypoint in mission_waypoints:
                    print("Waypoint already saved")
                    continue

                mission_waypoints.append(current_waypoint)

                continue

            elif type(event.key) == KeyCode:
                continue

            elif event.key.name == 'up':
                print("up")
                front_waypoint = generate_front_waypoint(current_location)
                WayMan.add_waypoint([front_waypoint, "front"])

                last_key_timestamp = time()
                continue
            elif event.key.name == 'down':
                print("down")
                back_waypoint = generate_back_waypoint(current_location)
                WayMan.add_waypoint([back_waypoint, "back"])

                last_key_timestamp = time()
                continue
            elif event.key.name == 'left':
                print("left")
                left_waypoint = generate_left_waypoint(current_location)
                WayMan.add_waypoint([left_waypoint, "left"])

                last_key_timestamp = time()
                continue
            elif event.key.name == 'right':
                print("right")
                right_waypoint = generate_right_waypoint(current_location)
                WayMan.add_waypoint([right_waypoint, "right"])

                last_key_timestamp = time()
                continue

            elif event.key.name == 'ctrl_r':
                print("following on waypoints")
                if len(WayMan.waypoints) == 0:
                    print("No waypoints")
                    #continue

                #WayMan.squash_waypoints()

                # TODO: Fix heading to be the same as the last waypoint
                #for waypoint in WayMan.waypoints:
                #    drone.goto(waypoint[0], 1)

                initial_waypoint = Waypoint(-35.3632609, 149.1652994, 588.1, 0)
                drone.goto(initial_waypoint, 1)
                target_waypoint = Waypoint(-35.3626562, 149.1652866, 587.84, 0)
                drone.goto(target_waypoint, 1)

                #WayMan.stash_waypoints()
                continue

            elif event.key.name == 'alt_r':
                print("Exiting & saving")
                WayMan.write_waypoints("waypoints.json")
                save_mission_waypoints()
                break



if __name__ == "__main__":
    # quit_on_controller_disconnect()
    try:
        #with keyboard.Events() as events:
        #    event = events.get(1)
        #    print(event.key.char)
        # WayMan.read_waypoints("waypoints.json")
        # main_controller()
        main_keyboard(False)
    except Exception as e:
        # WayMan.write_waypoints("waypoints.json")
        print(e)
        print(e.with_traceback)
        print("Exiting")
