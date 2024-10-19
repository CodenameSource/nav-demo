import math

from .util import pick_nearest_waypoint


# self.state = {
#    "position": np.zeros(3),  # TODO: Change format when switching to real coordinates
#    "collision": False,
#    "prev_observation": np.zeros(image_shape),
#    "prev_position": np.zeros(3),
#    "prev_action": -1,
#    "prev_offpath": False
# }

def interpret_action(action):
    if action == 0:
        return 1, 0  # Front
    elif action == 1:
        return 0, 1  # Right
    elif action == 2:
        return 0, -1  # Left
    elif action == 3:
        return -1, 0  # Back
    else:
        return 0, 0  # Hover or Following waypoints


def punish_repetition(position, prev_position, velocity, prev_action):  # TODO Improve strategy to allow for checking viable routes and returning if they are not
    """
     Punish the agent for repeating sequence of actions
    :param position: Dictionary of current position {x, y, z}, Position after current action
    :param prev_position:  Dictionary of previous position {x, y, z}, Position after prev_action
    :param prev_action:  Integer representing the previous action {0, 1, 2, 3, 4}
    :return:
    """
    x_change, y_change = interpret_action(prev_action)

    if (position['x'] - (prev_position['x'] - x_change)) <= velocity/2 and (
            position['y'] - (prev_position['y'] - y_change) <= velocity/2):
        return -2

    return 0


def punish_moving_away_from_target(position, prev_position, target_position, calculated_reward):
    """
    Punish the agent for moving away from the target position with respect to a certain threshold
    :param position: Dictionary of current position {x, y, z}
    :param prev_position:  Dictionary of previous position {x, y, z}
    :param target_position: Dictionary of target position {x, y, z}
    :return:
    """
    THRESHOLD = 0
    PUNISH_COEF = 1.05
    MIN_PUNISH = .5

    dist_to_target = math.sqrt(
        (position['x'] - target_position['x']) ** 2 + (position['y'] - target_position['y']) ** 2)
    prev_dist_to_target = math.sqrt(
        (prev_position['x'] - target_position['x']) ** 2 + (prev_position['y'] - target_position['y']) ** 2)

    if dist_to_target > prev_dist_to_target + THRESHOLD:
        return min(-PUNISH_COEF * abs(calculated_reward), -MIN_PUNISH), dist_to_target

    return 0, dist_to_target


def punish_off_path(position, target_position, waypoints, prev_offpath):
    """
    Punish the agent for moving off the planned path
    :param position: Dictionary of current position {x, y, z}
    :param target_position: Dictionary of target position {x, y, z}
    :param waypoints: List of waypoints [{x, y, z}]
    :return:
    """
    THRESHOLD = 2

    near_target = pick_nearest_waypoint(waypoints, position, target_position)
    dist_to_nearest_waypoint = math.sqrt(
        (position['x'] - near_target['x']) ** 2 + (position['y'] - near_target['y']) ** 2)

    if dist_to_nearest_waypoint > THRESHOLD:
        if prev_offpath:
            return -10, dist_to_nearest_waypoint, near_target
        else:
            return -5, dist_to_nearest_waypoint, near_target

    return 0, dist_to_nearest_waypoint, near_target

def apply_bias_towards_action(action, biased_action):
    REWARD = 3

    if action == biased_action:
        return REWARD

    return 0
