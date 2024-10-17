import math

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

def pick_nearest_waypoint(waypoints, current_pos, target_pos):
    min_dist = float('inf')
    nearest_waypoint = None
    dist_to_target = math.sqrt((current_pos['x'] - target_pos['x'])**2 + (current_pos['y'] - target_pos['y'])**2)

    for waypoint in waypoints:
        dist = math.sqrt((current_pos['x'] - waypoint['x'])**2 + (current_pos['y'] - waypoint['y'])**2)
        waypoint_to_target = math.sqrt((waypoint['x'] - target_pos['x'])**2 + (waypoint['y'] - target_pos['y'])**2)

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

    num_waypoints = int(math.sqrt(x_dist**2 + y_dist**2) / waypoint_dist)

    for i in range(num_waypoints):
        x = start['x'] + i * x_dist / num_waypoints
        y = start['y'] + i * y_dist / num_waypoints
        waypoints.append({'x': x, 'y': y, 'z': z, 'pitch': 0, 'roll': 0, 'yaw': 0})

    return waypoints