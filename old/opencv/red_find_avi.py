import numpy as np
import cv2 as cv
import sys
import time
import datetime
import socket

#cap = cv.VideoCapture('0420_run1.avi')
cap = cv.VideoCapture(0)

cap_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
cap_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
print(cap_width)

fourcc = cv.VideoWriter_fourcc('X', 'V', 'I', 'D')
out = cv.VideoWriter('./output.avi', fourcc, 20, (cap_width, cap_height))

lower_bound1 = np.array([0, 70, 50])
upper_bound1 = np.array([10, 255, 255])

lower_bound2 = np.array([170, 70, 50])
upper_bound2 = np.array([180, 255, 255])

kernel = np.ones((7, 7), np.uint8)

print ('Video stream opened using OpenCV version: ' + cv.__version__ + '\n')

counter = 0
# Initialization
heart_str = str.encode("Thump Thump")
done_str = str.encode("Done")

socket_info2 = ("127.0.0.1", 20002)
udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp_socket.bind(socket_info2)

start_time = datetime.datetime.utcnow() # When script started.
time_elapsed = datetime.datetime.utcnow() - start_time
heart_beat = datetime.datetime.utcnow()

camera_out = False

while True:

    if cv.waitKey(1) == ord('q'):
        break
        
    time_elapsed = datetime.datetime.utcnow() - start_time
    heart_elapsed = datetime.datetime.utcnow() - heart_beat 

    if (heart_elapsed.total_seconds() > 3):
      print("Sending a heartbeat.")
      heart_beat = datetime.datetime.utcnow()
      my_bytes = bytearray()
      my_bytes.append(0xFF)
      my_bytes.append(0xFF)
      my_bytes.append(0xFF)
      my_bytes.append(0xFF)
      my_bytes.append(0xFF)
      my_bytes.append(0xFF)
      udp_socket.sendto(my_bytes, ("127.0.0.1", 20001))
      
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        continue
        
    frame = cv.transpose(frame)
    frame = cv.flip(frame, 0)
    frame = cv.resize(frame, (cap_width, cap_height))
    hsv2 = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask2 = cv.inRange(hsv2, lower_bound1, upper_bound1)
    mask3 = cv.inRange(hsv2, lower_bound2, upper_bound2)
    maskor = cv.bitwise_or(mask2, mask3)
    invmask = cv.bitwise_not(maskor)

    #mask2 = cv.morphologyEx(mask2, cv.MORPH_CLOSE, kernel)
    #mask2 = cv.morphologyEx(mask2, cv.MORPH_OPEN, kernel)

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

    detector = cv.SimpleBlobDetector_create(params)
    keypoints = detector.detect(invmask)
    blank = np.zeros((1,1))
    blobs = cv.drawKeypoints(frame, keypoints, blank, (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    #blobs = cv.putText(blobs, 'balloon', (x, y), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv.LINE_AA)
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
      
  
    out.write(blobs)
    cv.imshow('frame', blobs)

# When everything done, release the capture
udp_socket.close()
cap.release()
out.release()
cv.destroyAllWindows()

exit()
