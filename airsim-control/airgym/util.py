import math
import numpy as np


def haversine(lat1, lon1, lat2, lon2):
    # Radius of the Earth in kilometers
    R = 6371.0

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Difference in coordinates
    delta_lat = lat2_rad - lat1_rad
    delta_lon = lon2_rad - lon1_rad

    # Haversine formula
    a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in kilometers
    distance = R * c

    return distance


def lat_distance(lat1, lat2):
    # Radius of the Earth in kilometers
    R = 6371.0

    # Convert latitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)

    # Difference in latitude
    delta_lat = lat2_rad - lat1_rad

    # Distance only based on latitude difference
    a = math.sin(delta_lat / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in kilometers along the same meridian
    distance = R * c

    return distance


def lon_distance(lon1, lon2, latitude):
    # Radius of the Earth in kilometers
    R = 6371.0

    # Convert longitude and latitude from degrees to radians
    lon1_rad = math.radians(lon1)
    lon2_rad = math.radians(lon2)
    lat_rad = math.radians(latitude)

    # Difference in longitude
    delta_lon = lon2_rad - lon1_rad

    # Distance only based on longitude difference (taking latitude into account)
    a = math.cos(lat_rad) * math.sin(delta_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in kilometers along the same parallel
    distance = R * c

    return distance


def new_latitude_with_heading(lat, delta_meters, heading):
    # Earth's radius in meters
    R = 6371000

    # Convert latitude and heading to radians
    lat_rad = math.radians(lat)
    heading_rad = math.radians(heading)

    # Convert delta_meters to angular distance in radians
    angular_distance = delta_meters / R

    # Calculate new latitude in radians
    new_lat_rad = math.asin(math.sin(lat_rad) * math.cos(angular_distance) +
                            math.cos(lat_rad) * math.sin(angular_distance) * math.cos(heading_rad))

    # Convert back to degrees
    new_lat = math.degrees(new_lat_rad)

    return new_lat


def new_longitude_with_heading(lat, lon, delta_meters, heading):
    # Earth's radius in meters
    R = 6371000

    # Convert latitude, longitude, and heading to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    heading_rad = math.radians(heading)

    # Convert delta_meters to angular distance in radians
    angular_distance = delta_meters / R

    # Calculate new latitude first (needed for longitude calculation)
    new_lat_rad = math.asin(math.sin(lat_rad) * math.cos(angular_distance) +
                            math.cos(lat_rad) * math.sin(angular_distance) * math.cos(heading_rad))

    # Calculate new longitude in radians
    new_lon_rad = lon_rad + math.atan2(math.sin(heading_rad) * math.sin(angular_distance) * math.cos(lat_rad),
                                       math.cos(angular_distance) - math.sin(lat_rad) * math.sin(new_lat_rad))

    # Convert back to degrees
    new_lon = math.degrees(new_lon_rad)

    return new_lon


def calculate_target_heading_cartesian(current_pos, target_pos):
    """
    Calculates the target heading (bearing) from the current location {x, y} to the target location {x, y}.

    Args:
        current_x (float): Current x-coordinate (analogous to latitude).
        current_y (float): Current y-coordinate (analogous to longitude).
        target_x (float): Target x-coordinate (analogous to latitude).
        target_y (float): Target y-coordinate (analogous to longitude).

    Returns:
        float: Target heading in degrees from the current location towards the target location (0-360 degrees).
    """
    # Calculate differences in coordinates
    delta_x = target_pos['x'] - current_pos['x']
    delta_y = target_pos['y'] - current_pos['y']

    # Calculate the target heading (in radians)
    heading_radians = np.arctan2(delta_y, delta_x)

    # Convert heading from radians to degrees
    heading_degrees = np.degrees(heading_radians)

    # Normalize the heading to be in the range 0-360 degrees
    target_heading = (heading_degrees + 360) % 360

    return target_heading

def quaternion_to_euler(quaternion):
    """
    Converts quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
    Returns yaw in degrees.
    """
    x, y, z, w = quaternion.x_val, quaternion.y_val, quaternion.z_val, quaternion.w_val

    # Yaw (z-axis rotation, this is the heading)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Convert yaw from radians to degrees
    yaw_degrees = math.degrees(yaw)

    return yaw_degrees


def get_drone_heading(client):
    """
    Retrieves the drone's heading (yaw) in degrees from AirSim.

    Args:
        client: AirSim client object.

    Returns:
        float: Drone's heading (yaw) in degrees.
    """
    # Get the drone's pose (position and orientation)
    drone_state = client.getMultirotorState()
    orientation_quaternion = drone_state.kinematics_estimated.orientation

    # Convert quaternion to Euler angles and get the yaw (heading)
    heading = quaternion_to_euler(orientation_quaternion)

    return max(heading, 0) #Quick fix, TODO: Find why sometimes the heading is negative

def pick_nearest_waypoint(waypoints, current_pos, target_pos):
    min_dist = float('inf')
    nearest_waypoint = None
    dist_to_target = math.sqrt((current_pos['x'] - target_pos['x']) ** 2 + (current_pos['y'] - target_pos['y']) ** 2)

    for waypoint in waypoints:
        dist = math.sqrt((current_pos['x'] - waypoint['x']) ** 2 + (current_pos['y'] - waypoint['y']) ** 2)
        waypoint_to_target = math.sqrt((waypoint['x'] - target_pos['x']) ** 2 + (waypoint['y'] - target_pos['y']) ** 2)

        if dist < min_dist and waypoint_to_target < dist_to_target:
            min_dist = dist
            nearest_waypoint = waypoint

    if nearest_waypoint is None:
        return target_pos
    return nearest_waypoint


def generate_waypoints(start, finish, z=-5, waypoint_dist=1):
    waypoints = []
    x_dist = finish['x'] - start['x']
    y_dist = finish['y'] - start['y']

    num_waypoints = int(math.sqrt(x_dist ** 2 + y_dist ** 2) / waypoint_dist)

    for i in range(num_waypoints):
        x = start['x'] + i * x_dist / num_waypoints
        y = start['y'] + i * y_dist / num_waypoints
        waypoints.append({'x': x, 'y': y, 'z': z, 'pitch': 0, 'roll': 0, 'yaw': 0})

    return waypoints
