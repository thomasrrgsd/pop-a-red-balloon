"""

Team A12: Pop A Red Balloon

Description: test1.py --connect <*connection_string>
** Temporary script that is going to eventually be navi.py **
This script connects to the drone and waits until armed. When armed it will takeoff
to 3m altitude, then navigate a 50m by 50m square. It sweeps the square in a grid
pattern. When it find a red object, it will stop and rotate the gimble to center the
red object in its view. The drone will align to this direction and begin navigating
forward and down to the red object. If the confidence level is high enough, it will
attempt to pop the object it thinks is a red balloon. If the confidence isn't high
enough, it will go back to what it was doing and mark that object as not worthy. If
it goes to pop the object, then it will look for the object to disappear with a gimbal
sweep. When that happens, it will return to home.

Preflight checks such as GPS lock and setting home location are handled by Ardupilot.
Additionally, low battery, geofence, and connection loss are handled by the Ardupilot.

"""

from __future__ import print_function

import 	math
import 	time
import 	sys
from 	dronekit	import connect, VehicleMode, LocationGlobalRelative
from 	pymavlink	import mavutil

sys.path.append("./func")
#from func_navi import *    # Contains all definitions for navi.py operation.
from generate_grid_waypoints import *

# Size of square in meters
SQUARE_SIZE = 10
# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 5

# Desired groundspeed in meters / second.
GROUND_SPEED = 0.5

# Set yaw of drone when sweeping. North 0, east 90, south 180, west 270.
SWEEP_YAW = 0
# Set how many 5 meter rows to sweep. Choose between 1 and 10.
SWEEP_NUM = 2

LONG_INIT = 35.727281
LATI_INIT = -78.696579

longitude, latitude = generate_grid_waypoints(SWEEP_YAW, SWEEP_NUM, LONG_INIT, LATI_INIT)

# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1

# Variable to keep track of if joystick to arm has returned to center
rcin_4_center = False

def yaw_behavior ():
  msg = vehicle.message_factory.param_request_list_encode(1,1)
  vehicle.send_mavlink(msg)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distanceToWaypoint(coordinates):
    """
    Returns distance between vehicle and specified coordinates
    """
    distance = get_distance_metres(vehicle.location.global_frame, coordinates)
    return distance

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

    # delay to wait until yaw of copter is at desired yaw angle
    time.sleep(3)

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

# This is where our code begins for right now.

# Have drone yaw in position of travel to begin with.
vehicle.parameters['WP_YAW_BEHAVIOR'] = 3
print('Going to waypoint 0.')

targetLocation = LocationGlobalRelative(longitude[0], latitude[0], TARGET_ALTITUDE)
vehicle.simple_goto(targetLocation, groundspeed = GROUND_SPEED)

while(distanceToWaypoint(targetLocation) > WAYPOINT_LIMIT):
   time.sleep(1)

# Set the drone yaw and give it time to get to that yaw. 2 seconds should be plenty.
condition_yaw(SWEEP_YAW, False)
time.sleep(2)
vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
time.sleep(2)

# Change it up
for i in range(1, len(longitude)):
  # When a certain key press is found, drone stops, goes to spot then comes back to resume sweep.
  # Another key press causes the drone to RTL. Once this works, we will put drone in loop.
  
  print('Going to waypoint %d.' % i)
  targetLocation = LocationGlobalRelative(longitude[i], latitude[i], TARGET_ALTITUDE)
  vehicle.simple_goto(targetLocation, groundspeed = GROUND_SPEED)
   
  while(distanceToWaypoint(targetLocation) > WAYPOINT_LIMIT):
    time.sleep(1)
  
  time.sleep(1)

# Allow drone to face home when going home.
vehicle.parameters['WP_YAW_BEHAVIOR'] = 3
time.sleep(1)

print('Returning to launch.')
vehicle.mode = VehicleMode("RTL")

# Stay connected to vehicle until landed and disarmed
while vehicle.armed:
    time.sleep(1)

print("Vehicle disarmed, the script will now exit.")

# Close vehicle object before exiting script
vehicle.close()
