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
from 	waypoints	import *
from 	dronekit	import connect, VehicleMode, LocationGlobalRelative
from 	pymavlink	import mavutil

#sys.path.append("./func")
#from func_navi import *    # Contains all definitions for navi.py operation.

# Size of square in meters
SQUARE_SIZE = 10
# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 5

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

def goto_yaw (location, airspeed=None, groundspeed=None):
       
        if isinstance(location, LocationGlobalRelative):
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            alt = location.alt
        elif isinstance(location, LocationGlobal):
            # This should be the proper code:
            # frame = mavutil.mavlink.MAV_FRAME_GLOBAL
            # However, APM discards information about the relative frame
            # and treats any alt value as relative. So we compensate here.
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            if not vehicle.home_location:
                vehicle.commands.download()
                vehicle.commands.wait_ready()
            alt = location.alt - vehicle.home_location.alt
        else:
            raise ValueError('Expecting location to be LocationGlobal or LocationGlobalRelative.')

        vehicle._master.mav.mission_item_send(0, 0, 0, frame,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                           0, 0, math.nan, location.lat, location.lon,
                                           alt)

        if airspeed is not None:
            vehicle.airspeed = airspeed
        if groundspeed is not None:
            vehicle.groundspeed = groundspeed 

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

def get_location_metres(original_location, dNorth, dEast, altitude):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobalRelative(newlat, newlon, altitude)

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
GPS_1M = 1e-5 
LONG_INIT = 35.727312 + 50*GPS_1M
LATI_INIT = -78.696101 - 100*GPS_1M
longitude = [] 
latitude = []

#longitude.append(LONG_INIT)
#latitude.append(LATI_INIT)

# Waypoint initialization for grid sweep pattern.
for i in range(0, 6):
  longitude.append(LONG_INIT - 10*GPS_1M*i)
  longitude.append(LONG_INIT - 10*GPS_1M*i)
  if((i % 2) == 0):
    latitude.append(LATI_INIT)
    latitude.append(LATI_INIT + 50*GPS_1M)
  else:
    latitude.append(LATI_INIT + 50*GPS_1M)
    latitude.append(LATI_INIT)


vehicle.parameters['WP_YAW_BEHAVIOR'] = 3
# How do i get yaw feedback?
# Set the yaw and wait for it to get there and then go to first waypoint.
# Go to waypoint 1 and then set appropriate yaw.
print('Going to waypoint 0.')

time.sleep(2)

targetLocation = LocationGlobalRelative(longitude[0], latitude[0], TARGET_ALTITUDE)
vehicle.simple_goto(targetLocation)
#goto_yaw(targetLocation)

while(distanceToWaypoint(targetLocation) > WAYPOINT_LIMIT):
   time.sleep(1)

condition_yaw(0, False)
time.sleep(2)

vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
time.sleep(2)

# Change it up
for i in range(1, len(longitude)):
   # When a certain key press is found, drone stops, goes to spot then comes back to resume sweep.
   # Another key press causes the drone to RTL. Once this works, we will put drone in loop.
   print('Going to waypoint %d.' % i)
   targetLocation = LocationGlobalRelative(longitude[i], latitude[i], TARGET_ALTITUDE)
   vehicle.simple_goto(targetLocation)
   #goto_yaw(targetLocation)
   while(distanceToWaypoint(targetLocation) > WAYPOINT_LIMIT):
      time.sleep(1)
   time.sleep(1)

# Go home. Need to determine how to use RTL and put into land mode.
print('Going to home location.')

vehicle.parameters['WP_YAW_BEHAVIOR'] = 3

time.sleep(2)

print('Returning to Launch')
vehicle.mode = VehicleMode("RTL")

# Stay connected to vehicle until landed and disarmed
while vehicle.armed:
    time.sleep(1)

print("Done!")

# Close vehicle object before exiting script
vehicle.close()
