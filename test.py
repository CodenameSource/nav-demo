from time import sleep

from Smav import Drone, Waypoint


host = '127.0.0.1' #tcp

drone = Drone(host, 5760)

#
#drone.goto(next_waypoint)
## TODO add waiting until at waypoint to prevent weird behavior
#sleep(15)
#next_waypoint = Waypoint(position['lat'], position['lon'] - .001, position['alt'] + 5)
#drone.goto(next_waypoint)
#sleep(2)
#drone.land_bellow()
drone.set_mode('GUIDED')
sleep(2)
drone.arm()
sleep(2)
drone.takeoff(5)
sleep(10)
position = drone.get_location()
next_waypoint = Waypoint(position['lat'], position['lon'] - .001, position['alt'] + 5)
drone.goto(next_waypoint)
drone.land_bellow()