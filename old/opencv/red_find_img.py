import numpy as np
import cv2 as cv
import sys
import time

lower_bound1 = np.array([0, 70, 50])
upper_bound1 = np.array([10, 255, 255])

lower_bound2 = np.array([170, 70, 50])
upper_bound2 = np.array([180, 255, 255])

kernel = np.ones((7, 7), np.uint8)

print ('\n Video stream opened using OpenCV version: ' + cv.__version__ + '\n')


img = cv.imread("pic1.png", cv.IMREAD_COLOR)
hsv2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
mask2 = cv.inRange(hsv2, lower_bound1, upper_bound1)
invmask = cv.bitwise_not(mask2)
#mask2 = cv.inRange(hsv2, lower_bound2, upper_bound2)
#mask2 = cv.morphologyEx(mask2, cv.MORPH_CLOSE, kernel)
#mask2 = cv.morphologyEx(mask2, cv.MORPH_OPEN, kernel)

params = cv.SimpleBlobDetector_Params()
detector = cv.SimpleBlobDetector_create(params)
keypoints = detector.detect(invmask)
blank = np.zeros((1,1))
blobs = cv.drawKeypoints(img, keypoints, blank, (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

x = int(keypoints[0].pt[0])
y = int(keypoints[0].pt[1])

print(x)
print(y)
blobs = cv.putText(blobs, 'balloon', (x, y), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv.LINE_AA)

cv.imshow("hello", blobs)

cv.waitKey(0)

cv.imshow("hello", img)

cv.waitKey(0)

cv.imshow("hello", hsv2)

cv.waitKey(0)

cv.imshow("hello", invmask)

cv.waitKey(0)

cv.imshow("hello", img)

cv.waitKey(0)

exit()
