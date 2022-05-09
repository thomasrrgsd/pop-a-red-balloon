"""

Succesful run completed with GitHub commit: aeaefc499b47a17f44b13faf8e44ce02b4479059

Team A12: Pop A Red Balloon

Description: navigation.py

This script sets the drone off on its grid sweep and starts the video feed, videoproc.py. The
video feed sends messages over a udp socket when a red object is detected. Every 3 seconds a
heartbeat is received from videoproc.py. If the drone finishes the grid sweep without receiving
a message to tell it a red object is found, it will return to launch.

If a message indicating a red object is received, the script saves the location at that instant,
and a flag is set to indicate that a red object was found. The drone drifts past the location and
so the drone is directed to go back to the saved location. A 5 second pause was sufficient for the
drone to return. The grid sweep is broken and the script moves on to the slow approach.

The slow approach is not overly complicated. If time permitted, this is where the most of our
improvements and optimizations would take place.

First note, we didn't have time to fix the socket overfilling. So we simply close and reopen it each
iteration. This is brute force and inefficient, dont do this. We got away with it because our system
responds and reacts slowly. The socket is always filling up due to the balloon being in frame constantly.

The slow approach socket is blocking and so the drone does not respond until new data is sent over the
udp socket. When a red object is detected, the position on the screen and its size is given. The drone
will yaw, lower altitude, or move forward to keep the balloon in the center of the screen. When the
size of the object is greater than 150, we know that the red object (balloon) is slightly forward
and beneath the popping mechanism. When centered and greater than 150 in size, the drone is pitched
forward and pops the balloon. The balloon then goes through the process of verifying the balloon is gone.

Whether it pops the balloon or loses the balloon due to wind or overcompensation, the drone takes 3
steps to verify the balloon is still around. First it goes back to the last location it saw the balloon.
Usually if it looses track of the balloon, it will refind it with this action. Next, it will return
to the location it first saw the red balloon during the grid sweep. This will happen if the balloon isn't
seen in the first scenario. The chances of the drone not being able to detect an unpopped balloon from
this position is very unlikely. If it does not detect the balloon in this position then it is very
likely the balloon is popped. It then returns to launch.

To be implented:
Have this script send commands to videoproc.py.

"""

from __future__ import print_function

import 	math
import 	time
import 	sys
import  os
import  socket
import  errno
from 	dronekit	import connect, VehicleMode, LocationGlobalRelative
from 	pymavlink	import mavutil

sys.path.append("./func")
from func_navi import *

def navigation (vehicle, longitude, latitude, SWEEP_YAW, TARGET_ALTITUDE, GROUND_SPEED, WAYPOINT_LIMIT):

  socket_info = ("127.0.0.1", 20001)  # Socket to rx from videoproc.py
  socket_info2 = ("127.0.0.1", 20002) # Socket to tx to videoproc.py
  udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
  udp_socket.bind(socket_info) 
  udp_socket.setblocking(0)  # non-blocking
  print("Socket on port 20001 is now open.")

  # Here is where we will call video processing script.
  # Opens in a new terminal because it utilizes stdout heavily.
  print("Opening video feed in seperate terminal.")
  os.system("gnome-terminal -e \"python3.7 ./videoproc.py\"")
  print("Giving video feed a couple of seconds to start.")
  time.sleep(2)
  
  # Flag for red object found.
  found = False

  # Cycle through grid waypoints.
  for i in range(1, len(longitude)):

    # Found red object, break grid sweep.
    if(found):
      break

    print('Going to waypoint %d.' % i)
    targetLocation = LocationGlobalRelative(longitude[i], latitude[i], TARGET_ALTITUDE)
    vehicle.simple_goto(targetLocation, groundspeed = GROUND_SPEED)
 
    while(distanceToWaypoint(vehicle, targetLocation) > WAYPOINT_LIMIT):
      
      # Message contains either the general heartbeat or if the red balloon 
      # is in frame it has the x and y cordinates of the ballon and its size
      # as it appears on the screen.
      time.sleep(1)

      # Exception handling is required for a non-blocking socket.
      try:
        # Message recieved, parse data.
        # Beautiful code.
        message, address = udp_socket.recvfrom(1024)
        my_bytes = bytearray(message)
        x = 0 | (my_bytes[0] << 8)
        x = x | (my_bytes[1])
        y = 0 | (my_bytes[2] << 8)
        y = y | (my_bytes[3]) 
        z = 0 | (my_bytes[4] << 8)
        z = z | (my_bytes[5])
        print("(x, y, diameter) = (%d, %d, %d)" % (x, y, z))

        # Sends a response back to opencv for acknowledgement.
        # Not implemented.
        #udp_socket.sendto(my_bytes, socket_info2)
        
        if((x == 65535) and (y == 65535)):
          print("Received a heart beat.")
          continue 

        #continue # Uncomment this code to skip responding to red object detected.

        # The only time we should receive here is when a red object is found.
        print("We found a red object!")
        print("Saving location...")
        det_loc = vehicle.location.global_relative_frame # Where red object first detected.
        lastest_balloon_location = det_loc # The latest location the balloon was detected.
        print("Location saved, breaking sweep.")
        print("Going to where balloon was found.")
        vehicle.simple_goto(det_loc, groundspeed = GROUND_SPEED)
        while(distanceToWaypoint(vehicle, det_loc) > WAYPOINT_LIMIT):
          time.sleep(1)
        print("Should be seeing balloon now, maybe, hopefully.")
        print("Starting a 5 second pause for stability.")
        time.sleep(5)
        print("Drone should be stopped in front of red balloon now.")
        # Done with grid sweep, moving on to slow approach.
        found = True
        print("Found flag set True, moving on to slow-approach.")
        break
      except socket.error, e: # Check the error flag thrown, make sure its data unavailable.
        err = e.args[0]
        if (err == errno.EAGAIN) or (err == errno.EWOULDBLOCK): # No data yet.
          print("No data from socket yet.")
          continue
        else: # Uh-oh. Mark, RTL.
          print("Unknown socket error, exiting navigation and closing socket.")
          udp_socket.close()
          return 1
    # End while loop
  # End for loop

  # For loop finished without finding red object.
  if(not found):
    print("Did not find red object during sweep, setting up for RTL.")
    udp_socket.close()
    return 2

###################################################
#
# Slow-approach:
#
##################################################

  current_yaw =0
  
  # when moving forward set velocity that drone moves on x axis
  # x velocity is a float
  x_forward_velocity = 0.3 # Approximately 1 ft/sec
  redTimeoutCounter = 0
  
  while (True):
      # Force close and open socket to clear out buffer.
      udp_socket.close()
      print("Socket has been closed.")
      socket_info = ("127.0.0.1", 20001)
      udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
      udp_socket.bind(socket_info)
      print("Socket has been opened. And is no longer non-blocking.")
      
      message, address = udp_socket.recvfrom(1024)
      my_bytes = bytearray(message)

      # This piece of code is beautiful. No matter what they say.
      # Words can't bring me dooooowwwwwnnnnnnn.
      x = 0 | (my_bytes[0] << 8)
      x = x | (my_bytes[1])
      y = 0 | (my_bytes[2] << 8)
      y = y | (my_bytes[3])
      z = 0 | (my_bytes[4] << 8)
      z = z | (my_bytes[5])
      print("( %d, %d, %d )" % (x, y, z))
      
      # We are now checking position on screen.
      if((x != 65535) and (y != 65535)):
        
        redTimeoutCounter = 0 # Reset heartbeat counter for balloon missing.
        current_location = vehicle.location.global_relative_frame # Save current location to modify altitude.
	
        # Save the lastest location where the drone found the balloon, save the yaw as well.
        lastest_balloon_location = vehicle.location.global_relative_frame
        latest_balloon_yaw = (vehicle.attitude.yaw)*(360/6.28)
        
        # Absolute deals in positive degrees, from 0 to 360.
        if(latest_balloon_yaw < 0):
          latest_balloon_yaw = 360 + latest_balloon_yaw
        
        if(x > 420): 
          condition_yaw_ccw_cw(vehicle, current_yaw+5, 1, True) # Yaw 5 degrees clockwise.
        if(x < 220): 
          condition_yaw_ccw_cw(vehicle, current_yaw+5, -1, True) # Yaw 5 degrees counterclockwise.
        time.sleep(1) 
        
        if(y > 300):
          # Altitude down.
          if(current_location.alt - 0.4 < 1.8): # Don't go to low, we know balloon is at least 6 ft up.
            print("Error, altitude is getting to low.")
          else:
            print("Lower altitude to bring object back in view.")
            current_location.alt = current_location.alt - 0.4
            vehicle.simple_goto(current_location, groundspeed = GROUND_SPEED)
        if(y < 180):
          # Altitude up. This should not be required, instead go forward.
          if(current_location.alt + 0.4 > 7):
            print("Error, altitude somehow managed to increase to 7 meters???")
          else:
            print("Moving forward to drop object in frame.")
            goto_position_target_local_ned(vehicle, x_forward_velocity)
            time.sleep(2)
            goto_position_target_local_ned(vehicle, 0)

            #current_location.alt = current_location.alt + 0.4
            #vehicle.simple_goto(current_location, groundspeed = GROUND_SPEED)
        time.sleep(1) 
        
        if((220 < x < 420) and (180 < y < 300)):
          # Check and see if balloon is too close.
          # Value is based on testing in the lab.
          if(z > 150):
            print("within popping range. Jogging forward at 2 m/s.")
            goto_position_target_local_ned(vehicle, 2)
            time.sleep(2)
            goto_position_target_local_ned(vehicle, 0)            
          else:    
            # Not close enough, go forward a  little bit.
            goto_position_target_local_ned(vehicle, x_forward_velocity)
            # Give drone two seconds to fly forward.
            time.sleep(2)
            # Tell drone to stop flying forward.
            goto_position_target_local_ned(vehicle, 0)
      
      # Heartbeat counter to count how long balloon has been missing from frame.
      if((x == 65535) and (y == 65535)):
	    redTimeoutCounter = redTimeoutCounter + 1
	    if(redTimeoutCounter == 3):
	      print("Camera has lost red object, let us go back to last location.")
	      print("Going to where balloon was LAST found.")
	      vehicle.simple_goto(lastest_balloon_location, groundspeed = GROUND_SPEED)
	      while(distanceToWaypoint(vehicle, lastest_balloon_location) > WAYPOINT_LIMIT):
	        time.sleep(1)
	      condition_yaw(vehicle, latest_balloon_yaw, False)
	      time.sleep(4)        
	      print("Should be seeing balloon now, maybe, hopefully.")
	
	    # If last location where balloon was found does not work
	    # go back to FIRST location where the balloon was found.
	    if(redTimeoutCounter == 6):
	      print("Balloon still not in camera view.")
	      print("Going to where balloon was FIRST found.")
	      vehicle.simple_goto(det_loc, groundspeed = GROUND_SPEED)
	      while(distanceToWaypoint(vehicle, det_loc) > WAYPOINT_LIMIT):
	        time.sleep(1)
	      condition_yaw(vehicle, SWEEP_YAW, False)
	      time.sleep(4)
	      print("Should be seeing balloon now, maybe, hopefully.")

            # That balloon ain't coming back.
	    if(redTimeoutCounter == 9):
	      print("We have truly lost track of balloon.")
	      print("Balloon is successfully popped, RTL.")
	      udp_socket.close()
	      return 0
        # End heart beat if statement
  # End while       

  udp_socket.close()
  return 0
# End function
