import airsim
import numpy as np
import gymnasium as gym
import cv2
from time import time

from util import generate_waypoints, pick_nearest_waypoint
from gymnasium import spaces

#TODO: Add documentation
#TODO: Refactor code
#TODO: Use real coordinates
class AirsimDroneEnv(gym.Env):
    #Inspired by https://github.com/microsoft/AirSim/blob/main/PythonClient/reinforcement_learning/airgym/envs/drone_env.py

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, ip_address: str, step_length: float, image_shape, start_position: dict, target_position: dict, velocity: float = .5, time_limit: int = 120):
        """

        :param ip_address:
        :param step_length:
        :param image_shape:
        :param start_position: dict with keys x, y, z, pitch, roll, yaw
        :param target_position: dict with keys x, y, z, pitch, roll, yaw
        :param velocity:
        :param time_limit: Time limit for the episode (In seconds)
        """

        self.image_shape = image_shape
        self.step_length = step_length
        self.start_position = start_position
        self.target_position = target_position
        self.velocity = velocity
        self.time_limit = time_limit
        self.start_time = time()

        self.waypoints = generate_waypoints(start_position, target_position) #TODO: Define waypoints

        self.observation_space = spaces.Box(low=0, high=255, shape=image_shape, dtype=np.uint8)
        self.action_space = spaces.Discrete(6)

        self.state = {
            "position": np.zeros(3), #TODO: Change format when switching to real coordinates
            "collision": False,
            "prev_observation": np.zeros(image_shape),
            "prev_position": np.zeros(3),
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
        start_pose = airsim.Pose(airsim.Vector3r(self.start_position['x'], self.start_position['y'], self.start_position['z']),
                                 airsim.to_quaternion(self.start_position['pitch'], self.start_position['roll'], self.start_position['yaw']))

        self.client.reset()
        self.client.simSetVehiclePose(start_pose, ignore_collision=True)
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.client.simGetCollisionInfo().has_collided
        self.client.takeoffAsync(1).join()
        self.client.moveToZAsync(-5, 1).join()
        self.start_time = time()



    def transform_img(self, img):
        MAX_DEPTH_DIST = 150 # Max distance for the estimated depth

        img1d = np.array(img.image_data_float, dtype=np.float32)
        img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
        img2d = np.reshape(img1d, (img.height, img.width))
        img2d = np.clip(img2d / MAX_DEPTH_DIST, 0, 1)

        img2d = cv2.resize(img2d, (self.image_shape[0], self.image_shape[1]), interpolation=cv2.INTER_AREA)
        img2d = img2d.reshape(self.image_shape[0], self.image_shape[1], 1)
        img2d = np.array(img2d * 255, dtype=np.uint8)

        #TODO: Resize image to self.image_shape
        return img2d

    def _get_obs(self):
        response = self.client.simGetImages([self.fpv_camera_feed])
        image = self.transform_img(response[0])

        return image

    def _get_state(self, prev_observation):
        current_pos = self.client.simGetVehiclePose().position
        current_pos = np.array([current_pos.x_val, current_pos.y_val, current_pos.z_val])
        colision = self.client.simGetCollisionInfo().has_collided

        self.state['prev_position'] = self.state['position']
        self.state['prev_observation'] = prev_observation
        self.state['position'] = current_pos
        self.state['collision'] = colision

    def _do_action(self, action):
        """

        :param action: {0: forward, 1: right, 2: left, 3: backward, 4: continue on predefined path, 5: hover}
        :return:
        """
        current_pos = self.state['position']
        current_pos = {'x': current_pos[0], 'y': current_pos[1], 'z': current_pos[2]}

        #TODO: Define current_pos format
        # Current waypoint format: {'x': x, 'y': y, 'z': z, 'pitch': pitch, 'roll': roll, 'yaw': yaw}
        if action == 0:
            #self.client.moveByVelocityAsync(1, 0, 0, 1).join()
            self.client.moveToPositionAsync(current_pos['x'] + self.step_length, current_pos['y'], current_pos['z'], self.velocity).join()
        elif action == 1:
            #self.client.moveByVelocityAsync(0, 1, 0, 1).join()
            self.client.moveToPositionAsync(current_pos['x'], current_pos['y'] + self.step_length, current_pos['z'], self.velocity).join()
        elif action == 2:
            #self.client.moveByVelocityAsync(0, -1, 0, 1).join()
            self.client.moveToPositionAsync(current_pos['x'], current_pos['y'] - self.step_length, current_pos['z'], self.velocity).join()
        elif action == 3:
            #self.client.moveByVelocityAsync(-1, 0, 0, 1).join()
            self.client.moveToPositionAsync(current_pos['x'] - self.step_length, current_pos['y'], current_pos['z'], self.velocity).join()
        elif action == 4:
            waypoint = pick_nearest_waypoint(self.waypoints, current_pos, self.target_position)
            self.client.moveToPositionAsync(waypoint['x'], waypoint['y'], waypoint['z'], self.velocity).join()

        elif action == 5:
            self.client.hoverAsync().join()

    def _compute_reward(self):
        threshold_dist = 2 # Distance threshold to planned path after which the drone is considered off path and being penalised for it
        dist_from_start_to_target = np.sqrt((self.start_position['x'] - self.target_position['x']) ** 2 + (
                    self.start_position['y'] - self.target_position['y']) ** 2)
        reward_for_reaching_target = 10
        added_bonus = 0
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
            near_target = pick_nearest_waypoint(self.waypoints, current_pos, self.target_position)
            dist_to_nearest_waypoint = np.sqrt((current_pos['x'] - near_target['x'])**2 + (current_pos['y'] - near_target['y'])**2)

            print(f"Distance to nearest waypoint: {dist_to_nearest_waypoint}")
            if dist_to_nearest_waypoint < threshold_dist:
                dist_to_target = np.sqrt((current_pos['x'] - self.target_position['x'])**2 + (current_pos['y'] - self.target_position['y'])**2)
                print(f"Distance to target: {dist_to_target}")

                prev_dist_to_target = np.sqrt(
                    (prev_pos['x'] - self.target_position['x']) ** 2 + (prev_pos['y'] - self.target_position['y']) ** 2)

                calculated_reward = reward_for_reaching_target - (dist_to_target_coef * dist_to_target)

                if dist_to_target < prev_dist_to_target:
                    added_bonus = .5
                else:
                    added_bonus = min(-1.05 * abs(calculated_reward), -.5)

                reward = calculated_reward + added_bonus
                print(f"Reward: {reward}, Added bonus: {added_bonus}")
            else:
                reward = -10
                truncated = True

        return reward, terminated, truncated

    def step(self, action):
        prev_observation = self.state['prev_observation']
        self._do_action(action)
        self._get_state(prev_observation)
        observation = self._get_obs()
        reward, terminated, truncated = self._compute_reward()
        return observation, reward, terminated, truncated, self.state

    def reset(self, seed=None, options=None):
        self._setup_flight()
        self._get_state(np.zeros(self.image_shape))
        return self._get_obs(), {}
