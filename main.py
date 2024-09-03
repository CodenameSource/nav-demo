import asyncio
from time import sleep

from flask import Flask, request, jsonify
from flask_cors import CORS

from Smav import Drone, Waypoint

host = '127.0.0.1'
port = 5760
app = Flask(__name__)
CORS(app)

waypoints = []

@app.route('/data', methods=['POST'])
def add_waypoints():
    data = request.get_json()
    for waypoint in data:
        waypoints.append(Waypoint(waypoint['lat'], waypoint['lng'], 580)) # TODO change altitude later

    return jsonify({'status': 'ok'})

def fix_waypoint_altitude(base_altitude, altitude):
    for waypoint in waypoints:
        waypoint.alt = base_altitude + altitude

def drone_connect_and_arm_proc():
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

    # Arm the drone again and wait for 4 seconds
    drone.arm()

    return drone

@app.route('/start', methods=['POST'])
async def start():
    # Initialize the drone with host and port
    drone = drone_connect_and_arm_proc()

    current_location = drone.get_location()
    fix_waypoint_altitude(current_location['alt'], current_location['relative_alt'] + 5)

    # Make the drone take off and wait for 10 seconds
    drone.takeoff(5)
    await asyncio.sleep(10)

    # Go to each waypoint
    for waypoint in waypoints:
        drone.goto(waypoint)

    drone.land_bellow()
    await asyncio.sleep(10)


    drone.disconnect()
    await asyncio.sleep(2)

    return jsonify({'status': 'ok'})

if __name__ == '__main__':
    app.run(debug=True)