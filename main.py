"""

Succesful run completed with GitHub commit: aeaefc499b47a17f44b13faf8e44ce02b4479059

Team A12: Pop A Red Balloon

Description: main.py --connect <*connection_string>
    Example: main.py --connect :14551

This script connects to the drone and waits until armed. When armed it will takeoff
to 5m altitude then navigate a 50m by 50m square. It sweeps the square in a grid
pattern. When it find a red object, it will stop and yaw to center the
red object in its view. The drone will align to this direction and begin navigating
forward and down to the red object. If it believes it has popped the object it will
return to the location it last saw the red object. If it no longer finds the red object
it will return to home.

Preflight checks such as GPS lock and setting home location are handled by Ardupilot.
Additionally, low battery, geofence, and connection loss are handled by the Ardupilot.

This script handles startup and takeoff and return to launch. Any mission actions are
performed in the navigation function.

"""

from __future__ import print_function

import 	math
import 	time
import 	sys
from 	dronekit	import connect, VehicleMode, LocationGlobalRelative
from 	pymavlink	import mavutil

sys.path.append("./func")
from func_navi import *               # All dronekit functions. Including those given by Sichitiu.
from generate_grid_waypoints import *
from navigation import *


# Desired altitude (in meters) to takeoff to.
TARGET_ALTITUDE = 5

# Desired groundspeed in meters / second.
GROUND_SPEED = 3

# Set yaw of drone when sweeping. North 0, east 90, south 180, west 270.
# Do not replace this variable as the grid is generated based on the yaw.
# Generating the grid and operating with different yaws will mess up the sweep.
SWEEP_YAW = 0
# Set how many 5 meter rows to sweep. Choose between 1 and 10.
SWEEP_NUM = 10

# Coordinates that are a little west of the usual launch pad position.
# A 50 meter by 50 meter square is drawn with this location at its top
# left corner. Make sure nothing is in the way. For example, the sprinkler.
LONG_INIT = 35.727281
LATI_INIT = -78.696800

# Creates a grid based on the drone absolute yaw and how many sweeps you want to make.
longitude, latitude = generate_grid_waypoints(SWEEP_YAW, SWEEP_NUM, LONG_INIT, LATI_INIT)

# Portion of TARGET_ALTITUDE at which we will break from takeoff loop.
ALTITUDE_REACH_THRESHOLD = 0.95

# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1

# Variable to keep track of if joystick to arm has returned to center.
# Used for takeoff.
rcin_4_center = False

# Set up option parsing to get connection string.
import argparse
parser = argparse.ArgumentParser(description='Main navigation script. Communicates with MAVProxy and video\
                                              processing application.')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

# Aquire the connection_string.
connection_string = args.connect

# Exit if no connection string specified.
if not connection_string:
    sys.exit('Please specify connection string, --connect :UDP_PORT')

# Connect to the vehicle. Dronekit command, has its own timeout.
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

print('Succesfully connected to vehicle.')

"""
Listens for RC_CHANNELS mavlink messages with the goal of determining when the RCIN_4 joystick
has returned to center for two consecutive seconds.
"""
@vehicle.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)


if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    vehicle.mode = VehicleMode("ALT_HOLD")

# Wait for pilot before proceeding.
print('Waiting for safety pilot to arm...')

# Wait until safety pilot arms drone.
while not vehicle.armed:
    time.sleep(1)

# Once you are in guided, you stay in guided. Mark does not want
# us changing modes in the middle of flight. If he sees the mode
# change then the mission is over.
print('Armed... now placing vehicle into guided mode.')
vehicle.mode = VehicleMode("GUIDED")
print('Vehicle is now in GUIDED mode.')

# Required for takeoff due to way that drone takes off with RC sticks.
# If message is recieved from RC Listener for 2 seconds, then the while
# loop breaks.
if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
    rcin_4_center_once = False
    rcin_4_center_twice = False
    while not rcin_4_center_twice:
        if rcin_4_center:
            if rcin_4_center_once:
                rcin_4_center_twice = True
            else:
                rcin_4_center_once = True
        else:
            rcin_4_center_once = False
        time.sleep(1)
    
    # Takeoff to 3m altitude
    print("Taking off to altitude of 5m.")
    vehicle.simple_takeoff(TARGET_ALTITUDE)  # Take off to target altitude.

    while True:
         # Break just below target altitude.
        if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            break
        time.sleep(0.5)

    print("We have reached our target altitude of 5m.")

# Have drone yaw in the direction of travel to begin with.
print("Yaw behavior is set to follow GPS coordinate.")
vehicle.parameters['WP_YAW_BEHAVIOR'] = 3
print('Going to waypoint 0.')

# This tells the drone to go to the first waypoint which is the top left corner of the grid.
targetLocation = LocationGlobalRelative(longitude[0], latitude[0], TARGET_ALTITUDE)
vehicle.simple_goto(targetLocation, groundspeed = GROUND_SPEED)
while(distanceToWaypoint(vehicle, targetLocation) > WAYPOINT_LIMIT):
   time.sleep(1)

# Set the drone yaw and give it time to get to that yaw. 2 seconds should be plenty.
# The yaw is fixed for the grid sweep to the SWEEP_YAW chosen at the top of this script.
condition_yaw(vehicle, SWEEP_YAW, False)
print("Setting yaw to desired sweep yaw position.")
time.sleep(2)
vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
print("Yaw is at desired sweep yaw position. Yaw behavior is now locked.")
time.sleep(1)

# Enters navigation function. When returning from this function, the mission is over and
# the drone is expected to be ready for RTL. The video feed is started inside this function.
navigation(vehicle, longitude, latitude, SWEEP_YAW, TARGET_ALTITUDE, GROUND_SPEED, WAYPOINT_LIMIT)


# Allow drone to face direction of travel when going home.
vehicle.parameters['WP_YAW_BEHAVIOR'] = 3
print("Drone yaw behavior now follows GPS navigation.")
time.sleep(1)

print('Returning to launch.')
vehicle.mode = VehicleMode("RTL")

# Stay connected to vehicle until landed and disarmed.
while vehicle.armed:
    time.sleep(1)

print("Vehicle disarmed, the script will now exit.")

# Close vehicle object before exiting script
vehicle.close()
sys.exit(0)
