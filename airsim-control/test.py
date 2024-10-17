import airsim
import cv2
import numpy as np
from PIL import Image
from time import sleep

start_pos = {"x": 8, "y": 6, "z": -1, "pitch": 0, "roll": 0, "yaw": 0}
end_pos = {"x": 70, "y": 6, "z": -5, "pitch": 0, "roll": 0, "yaw": 0}

def pick_nearest_waypoint(waypoints, current_pos, target_pos):
    min_dist = float('inf')
    nearest_waypoint = None
    dist_to_target = np.sqrt((current_pos['x'] - target_pos['x'])**2 + (current_pos['y'] - target_pos['y'])**2)

    for waypoint in waypoints:
        dist = np.sqrt((current_pos['x'] - waypoint['x'])**2 + (current_pos['y'] - waypoint['y'])**2)
        waypoint_to_target = np.sqrt((waypoint['x'] - target_pos['x'])**2 + (waypoint['y'] - target_pos['y'])**2)

        if dist < min_dist and waypoint_to_target < dist_to_target:
            min_dist = dist
            nearest_waypoint = waypoint

    if nearest_waypoint is None:
        return end_pos
    return nearest_waypoint

def generate_waypoints(start, finish, z=-5, waypoint_dist=1):
    waypoints = []
    x_dist = finish['x'] - start['x']
    y_dist = finish['y'] - start['y']

    num_waypoints = int(np.sqrt(x_dist**2 + y_dist**2) / waypoint_dist)

    for i in range(num_waypoints):
        x = start['x'] + i * x_dist / num_waypoints
        y = start['y'] + i * y_dist / num_waypoints
        waypoints.append({'x': x, 'y': y, 'z': z, 'pitch': 0, 'roll': 0, 'yaw': 0})

    return waypoints


img_type = airsim.ImageRequest(0, airsim.ImageType.DepthPerspective, True, False)

def visualise_img(img):
    img1d = np.array(img.image_data_float, dtype=np.float32)
    img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
    img2d = np.reshape(img1d, (img.height, img.width))
    img2d = np.clip(img2d / 200.0, 0, 1)

    img2d = cv2.resize(img2d, (84, 84), interpolation=cv2.INTER_AREA)
    img2d = img2d.reshape(84, 84, 1)

    cv2.imshow("Image", img2d)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

ip = "127.0.0.1"


sleep(2)
client = airsim.MultirotorClient(ip)
client.confirmConnection()
client.reset()
sleep(1)
client.enableApiControl(True)
client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(70, 6, -.5), airsim.to_quaternion(0, 0, 0)), True)
img = client.simGetImages([img_type])[0]
client.simGetCollisionInfo().has_collided
sleep(1)

current_pos = client.simGetVehiclePose().position
current_pos = {"x": current_pos.x_val, "y": current_pos.y_val, "z": current_pos.z_val}
print(current_pos)
client.takeoffAsync(1).join()
client.moveToZAsync(-5, 1).join()

client.moveToPositionAsync(current_pos['x'] + 3, current_pos['y'], current_pos['z'], 3,
                                vehicle_name='Copter').join()

#print(generate_waypoints(start_pos, end_pos))
visualise_img(img)
