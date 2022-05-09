"""

Succesful run completed with GitHub commit: aeaefc499b47a17f44b13faf8e44ce02b4479059

Team A12: Pop A Red Balloon

Description: videoproc.py

Must press and hold 'q' to end video feed.

This script takes in the video feed from the webcam frame by frame and looks for a red object.
The resulting analysis is displayed in real time and also saved to output.avi in the same folder.
When a red object is detected, a message is sent to navigation.py witht he objects screen position
and size.

Analysis:
1. The frame is rotated 90 deg to account for webcam orientation.
2. The frame is converted to HSV.
3. The red is filtered from the HSV frame with the first boundary condition and shows as white.
4. The red is filtered from the HSV frame with the second boundary condition and shows as white.
5. The results from the first and second boundary are combined.
6. The combined results is inverted to make the red show as black pixels.
7. A blob detector is used that detects collections of black pixels.
8. The location and size of the blob is sent to navigation.py if a blob is detected.

"""

import numpy as np
import cv2 as cv
import sys
import time
import datetime
import socket

# Use commented line to analyze pre-recorded videos. Useful for lab testing.
#cap = cv.VideoCapture('0420_run1.avi')

# Video camera feed is usually /dev/video0.
cap = cv.VideoCapture(0)

# Records the resolution to tell the avi file output later on the correct resolution.
# Without this we had the issue that the video file, output.avi, would be non-indexable.
cap_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
cap_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

# Setup avi file output.
fourcc = cv.VideoWriter_fourcc('X', 'V', 'I', 'D')
out = cv.VideoWriter('./output.avi', fourcc, 20, (cap_width, cap_height))

# Red color HSV boundaries.
# Red has two areas to check in the HSV spectrum. These values were used from a previous team.
# and were tweaked between runs at the field.
lower_bound1 = np.array([0, 70, 50])
upper_bound1 = np.array([10, 255, 255])
lower_bound2 = np.array([170, 70, 50])
upper_bound2 = np.array([180, 255, 255])

# Show the OpenCV version.
print ('Video stream opened using OpenCV version: ' + cv.__version__ + '\n')

# Initialization
socket_info  = ("127.0.0.1", 20001) # Tx to navigation.py.
socket_info2 = ("127.0.0.1", 20002) # Rx from navigation.py. Not implemented.
udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp_socket.bind(socket_info2)

# Timing used to coordinate heartbeats.
start_time = datetime.datetime.utcnow() # When script started.
time_elapsed = datetime.datetime.utcnow() - start_time
heart_beat = datetime.datetime.utcnow()

while True:

    # Checks for user to keypress 'q'. Video feed doesn't exit until this happens.
    if cv.waitKey(1) == ord('q'):
        break
       
    # Update heart beat timers. 
    time_elapsed = datetime.datetime.utcnow() - start_time
    heart_elapsed = datetime.datetime.utcnow() - heart_beat 
    if (heart_elapsed.total_seconds() > 3): # A heart beat is 3 seconds.
      print("Sending a heartbeat.")
      heart_beat = datetime.datetime.utcnow()
      my_bytes = bytearray()
      my_bytes.append(0xFF) # Heart been is represented by 65535, 65535, 65535.
      my_bytes.append(0xFF)
      my_bytes.append(0xFF)
      my_bytes.append(0xFF)
      my_bytes.append(0xFF)
      my_bytes.append(0xFF)
      udp_socket.sendto(my_bytes, ("127.0.0.1", 20001))
      
    # Capture frame-by-frame.
    ret, frame = cap.read()

    # If frame is read correctly ret is True.
    # If not True, we continue with loop in hopes that the feed restarts.
    # We had a loose USB so this was helpful for us, until we zip tied the USB in.
    # This lets the heartbeat continue.
    if not ret:
        print("Can't receive frame.")
        continue

    # First, the image must be rotated 90 deg because the webcam was mounted 90 deg rotated.  
    frame = cv.transpose(frame)
    frame = cv.flip(frame, 0)
    frame = cv.resize(frame, (cap_width, cap_height))

    # Next we convert the frame HSV valued frame.
    hsv2 = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # Mask2 and mask3 are the HSV valued frame filtered by each red hsv bound.
    mask2 = cv.inRange(hsv2, lower_bound1, upper_bound1)
    mask3 = cv.inRange(hsv2, lower_bound2, upper_bound2)
    # Combine the results for each red boundary.
    maskor = cv.bitwise_or(mask2, mask3)

    # The red comes through as white and the blob detector is based on dark pixels.
    # We must invert the image.
    invmask = cv.bitwise_not(maskor)

    # These are the parameters for the blob detector.
    # See software references for a link to the explanations.
    # These values were tweaked for better detection.
    params = cv.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 150
    params.maxArea = 300000
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
    # Filter by Convexity  
    params.filterByConvexity = True
    params.minConvexity = 0.87
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Create simple blob detector object with the above parameters.
    detector = cv.SimpleBlobDetector_create(params)

    # Apply detector to the inverted red hsv filtered frame.
    # Keypoints are the pixels that are found to be a part of a blob.
    keypoints = detector.detect(invmask)

    # Prints a red circle around the blob.
    blank = np.zeros((1,1))
    blobs = cv.drawKeypoints(frame, keypoints, blank, (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Add label to the detected object if you want.
    #blobs = cv.putText(blobs, 'balloon', (x, y), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv.LINE_AA)

    # A nonzero length keypoints tuple indicates that an object was found.
    # If the parameters are set right, we can say with confidence its our balloon.
    if(len(keypoints)):
      print("%d: Found red object." % counter)
      counter = counter + 1
      x = int(keypoints[0].pt[0])
      y = int(keypoints[0].pt[1])
      z = int(keypoints[0].size)
      my_bytes = bytearray()
      xlsb = (x & 0x00FF)
      xmsb = (x & 0xFF00) >> 8
      my_bytes.append(xmsb)
      my_bytes.append(xlsb)
      ylsb = (y & 0x00FF)
      ymsb = (y & 0xFF00) >> 8
      my_bytes.append(ymsb)
      my_bytes.append(ylsb)
      zlsb = (z & 0x00FF)
      zmsb = (z & 0xFF00) >> 8
      my_bytes.append(zmsb)
      my_bytes.append(zlsb)
      udp_socket.sendto(my_bytes, ("127.0.0.1", 20001))
    
    # Write frame to output.avi.
    out.write(blobs)
    # Show fram on screen.
    cv.imshow('frame', blobs)

# When everything is done, release the capture and close the socket.
udp_socket.close()
cap.release()
out.release()
cv.destroyAllWindows()

sys.exit(0)
