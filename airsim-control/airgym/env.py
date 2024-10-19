import airsim
import numpy as np
import gymnasium as gym
import cv2
from time import time

from .util import generate_waypoints, pick_nearest_waypoint, get_drone_heading, calculate_target_heading_cartesian
from .strategies import punish_repetition, punish_moving_away_from_target, punish_off_path, apply_bias_towards_action

from gymnasium import spaces


# TODO: Add documentation
# TODO: Refactor code
# TODO: Use real coordinates
class AirsimDroneEnv(gym.Env):
    # Inspired by https://github.com/microsoft/AirSim/blob/main/PythonClient/reinforcement_learning/airgym/envs/drone_env.py

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, ip_address: str, step_length: float, image_shape, start_position: dict, target_position: dict,
                 travel_velocity: float = 2, correction_velocity: float = .5, time_limit: int = 120):
        """

        :param ip_address:
        :param step_length:
        :param image_shape:
        :param start_position: dict with keys x, y, z, pitch, roll, yaw
        :param target_position: dict with keys x, y, z, pitch, roll, yaw
        :param correction_velocity:
        :param travel_velocity:
        :param time_limit: Time limit for the episode (In seconds)
        """

        self.image_shape = image_shape
        self.step_length = step_length
        self.start_position = start_position
        self.target_position = target_position
        self.correction_velocity = correction_velocity
        self.travel_velocity = travel_velocity
        self.time_limit = time_limit
        self.start_time = time()

        self.waypoints = generate_waypoints(start_position, target_position,
                                            waypoint_dist=1.5)  # TODO: Define waypoints
        self.move_count = 0

        #self.observation_space = spaces.Box(low=0, high=255, shape=image_shape, dtype=np.uint8)
        self.observation_space = spaces.Dict({
            'image': spaces.Box(low=0, high=255, shape=image_shape, dtype=np.uint8),
            'current_heading': spaces.Box(low=0, high=360, shape=(1,), dtype=np.float32),
            'distance_to_target': spaces.Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
            'target_heading': spaces.Box(low=0, high=360, shape=(1,), dtype=np.float32),
            'previous_image': spaces.Box(low=0, high=255, shape=image_shape, dtype=np.uint8),
            'previous_heading': spaces.Box(low=0, high=360, shape=(1,), dtype=np.float32),
            'previous_distance_to_target': spaces.Box(low=0, high=np.inf, shape=(1,), dtype=np.float32),
            'previous_target_heading': spaces.Box(low=0, high=360, shape=(1,), dtype=np.float32)
        })  # TODO: Add previous observation to the obs space
        self.action_space = spaces.Discrete(6)
        self.named_action_space = {"forward": 0, "right": 1, "left": 2, "backward": 3, "continue": 4, "hover": 5,
                                   0: "forward", 1: "right", 2: "left", 3: "backward", 4: "continue", 5: "hover"}

        self.state = {
            "position": np.zeros(3),  # TODO: Change format when switching to real coordinates
            "heading": 0,
            "distance_to_target": 0,
            "target_heading": 0,
            "collision": False,
            "prev_observation": np.zeros(image_shape, dtype=np.uint8),
            "prev_position": np.zeros(3),
            "prev_heading": 0,
            "prev_distance_to_target": 0,
            "prev_target_heading": 0,
            "prev_action": -1,
            "prev_offpath": False
        }

        self.client = airsim.MultirotorClient(ip=ip_address)
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        print(self.client.getMultirotorState())

        self._setup_flight()

        self.fpv_camera_feed = airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False)

    def __del__(self):
        self.client.reset()

    def _setup_flight(self):
        start_pose = airsim.Pose(
            airsim.Vector3r(self.start_position['x'], self.start_position['y'], self.start_position['z']),
            airsim.to_quaternion(self.start_position['pitch'], self.start_position['roll'], self.start_position['yaw']))

        self.client.reset()
        self.client.simSetVehiclePose(start_pose, ignore_collision=True)
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.client.simGetCollisionInfo().has_collided
        self.client.takeoffAsync(1).join()
        self.client.moveToZAsync(-5, 1).join()
        self.start_time = time()
        self.move_count = 0

    def transform_img(self, img):
        MAX_DEPTH_DIST = 150  # Max distance for the estimated depth

        img1d = np.array(img.image_data_float, dtype=np.float32)
        img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
        img2d = np.reshape(img1d, (img.height, img.width))
        img2d = np.clip(img2d / MAX_DEPTH_DIST, 0, 1)

        img2d = cv2.resize(img2d, (self.image_shape[0], self.image_shape[1]), interpolation=cv2.INTER_AREA)
        img2d = img2d.reshape(self.image_shape[0], self.image_shape[1], 1)
        img2d = np.array(img2d * 255, dtype=np.uint8)

        # TODO: Resize image to self.image_shape
        return img2d

    def _get_obs(self):
        response = self.client.simGetImages([self.fpv_camera_feed])

        image = self.transform_img(response[0])

        obs_dict = {
            'image': image,
            'current_heading':  np.array([self.state['heading']], dtype=np.float32),
            'distance_to_target': np.array([self.state['distance_to_target']], dtype=np.float32),
            'target_heading': np.array([self.state['target_heading']], dtype=np.float32),
            'previous_image': self.state['prev_observation'],
            'previous_heading': np.array([self.state['prev_heading']], dtype=np.float32),
            'previous_distance_to_target': np.array([self.state['prev_distance_to_target']], dtype=np.float32),
            'previous_target_heading': np.array([self.state['prev_target_heading']], dtype=np.float32)
        }

        #return image
        return obs_dict


    def _get_state(self):
        current_pos = self.client.simGetVehiclePose().position
        current_pos = np.array([current_pos.x_val, current_pos.y_val, current_pos.z_val])
        current_heading = get_drone_heading(self.client)
        current_dist_to_target = np.sqrt((current_pos[0] - self.target_position['x']) ** 2 + (
                current_pos[1] - self.target_position['y']) ** 2)
        target_heading = calculate_target_heading_cartesian({'x': current_pos[0], 'y': current_pos[1], 'z': current_pos[2]}, self.target_position)
        colision = self.client.simGetCollisionInfo().has_collided

        self.state['prev_position'] = self.state['position']
        self.state['prev_heading'] = self.state['heading']
        self.state['prev_distance_to_target'] = self.state['distance_to_target']
        self.state['prev_target_heading'] = self.state['target_heading']
        self.state['position'] = current_pos
        self.state['heading'] = current_heading
        self.state['distance_to_target'] = current_dist_to_target
        self.state['target_heading'] = target_heading
        self.state['collision'] = colision

    def _do_action(self, action):
        """

        :param action: {0: forward, 1: right, 2: left, 3: backward, 4: continue on predefined path, 5: hover}
        :return:
        """
        current_pos = self.state['position']
        current_pos = {'x': current_pos[0], 'y': current_pos[1], 'z': current_pos[2]}

        self.move_count += 1

        # TODO: Define current_pos format
        # Current waypoint format: {'x': x, 'y': y, 'z': z, 'pitch': pitch, 'roll': roll, 'yaw': yaw}
        if action == self.named_action_space['forward']:
            # self.client.moveByVelocityAsync(1, 0, 0, 1).join()
            self.client.moveToPositionAsync(current_pos['x'] + self.step_length, current_pos['y'], current_pos['z'],
                                            self.correction_velocity).join()
        elif action == self.named_action_space['right']:
            # self.client.moveByVelocityAsync(0, 1, 0, 1).join()
            self.client.moveToPositionAsync(current_pos['x'], current_pos['y'] + self.step_length, current_pos['z'],
                                            self.correction_velocity).join()
        elif action == self.named_action_space['left']:
            # self.client.moveByVelocityAsync(0, -1, 0, 1).join()
            self.client.moveToPositionAsync(current_pos['x'], current_pos['y'] - self.step_length, current_pos['z'],
                                            self.correction_velocity).join()
        elif action == self.named_action_space['backward']:
            # self.client.moveByVelocityAsync(-1, 0, 0, 1).join()
            self.client.moveToPositionAsync(current_pos['x'] - self.step_length, current_pos['y'], current_pos['z'],
                                            self.correction_velocity).join()
        elif action == self.named_action_space['continue']:
            waypoint = pick_nearest_waypoint(self.waypoints, current_pos, self.target_position)
            self.client.moveToPositionAsync(waypoint['x'], waypoint['y'], waypoint['z'], self.travel_velocity).join()
        elif action == self.named_action_space['hover']:
            self.client.hoverAsync().join()

    def _compute_reward(self, action):
        #TODO: Refactor reward computation to limit and normalise reward between lower and upper bound

        TERMINATION_REWARD = -20

        dist_from_start_to_target = np.sqrt((self.start_position['x'] - self.target_position['x']) ** 2 + (
                self.start_position['y'] - self.target_position['y']) ** 2)
        reward_for_reaching_target = 10
        dist_to_target_coef = reward_for_reaching_target / dist_from_start_to_target
        current_pos = self.state['position']
        current_pos = {'x': current_pos[0], 'y': current_pos[1], 'z': current_pos[2]}
        prev_pos = self.state['prev_position']
        prev_pos = {'x': prev_pos[0], 'y': prev_pos[1], 'z': prev_pos[2]}
        current_time = time()

        terminated = False
        truncated = False
        if self.state['collision']:
            reward = -100
            terminated = True
        elif current_time - self.start_time > self.time_limit:
            reward = -10
            truncated = True
        else:
            dist_to_target = np.sqrt((current_pos['x'] - self.target_position['x']) ** 2 + (
                    current_pos['y'] - self.target_position['y']) ** 2)
            reward = dist_to_target_coef * (dist_from_start_to_target - dist_to_target)

            OFF_PATH_PENALTY = False
            OFF_TARGET_PENALTY = False
            REPETITION_PENALTY = False

            reward_added, dist_to_nearest_waypoint, nearest_waypoint = punish_off_path(current_pos,
                                                                                       self.target_position,
                                                                                       self.waypoints,
                                                                                       self.state['prev_offpath'])
            reward += reward_added
            if reward_added < 0:  # If the agent is off path, set prev_offpath to True
                self.state['prev_offpath'] = True
                OFF_PATH_PENALTY = True
            else:
                self.state['prev_offpath'] = False

            reward_added, dist_to_target = punish_moving_away_from_target(current_pos, prev_pos, self.target_position,
                                                                          reward)
            reward += reward_added
            if reward_added < 0:
                OFF_TARGET_PENALTY = True

            reward_added = punish_repetition(current_pos, prev_pos, self.correction_velocity, self.state['prev_action'])
            reward += reward_added
            if reward_added < 0:
                REPETITION_PENALTY = True

            reward_added = apply_bias_towards_action(action, self.named_action_space['continue'])
            reward += reward_added

            print("--------------------")
            print(f"Move #{self.move_count}")
            print(f"Action: {self.named_action_space[action]}")
            print(f"Current position: {current_pos}")
            print(f"Nearest waypoint: {nearest_waypoint}")
            print(f"Distance to nearest waypoint: {dist_to_nearest_waypoint}")
            print(f"Distance to target: {dist_to_target}")
            print(
                f"Penalties: Off path: {OFF_PATH_PENALTY}, Off target: {OFF_TARGET_PENALTY}, Repetition: {REPETITION_PENALTY}")
            print(f"Reward: {reward}")

            if reward < TERMINATION_REWARD:
                terminated = True

        return reward, terminated, truncated

    def step(self, action):
        self._do_action(action)
        self._get_state()
        observation = self._get_obs()
        reward, terminated, truncated = self._compute_reward(action)
        self.state['prev_action'] = action
        self.state['prev_observation'] = observation['image'] #TODO: Make changes to state only in custom methods

        return observation, reward, terminated, truncated, self.state

    def reset(self, seed=None, options=None):
        self._setup_flight()
        self._get_state()
        return self._get_obs(), {}