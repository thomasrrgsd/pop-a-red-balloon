import numpy as np
import cv2 as cv
cap = cv.VideoCapture(0)
import sys
import time

#cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
#cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

cap_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
cap_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

lower_bound1 = np.array([0, 70, 50])
upper_bound1 = np.array([10, 255, 255])

lower_bound2 = np.array([170, 70, 50])
upper_bound2 = np.array([180, 255, 255])

kernel = np.ones((7, 7), np.uint8)

print(cap_width)
print(cap_height)

fourcc = cv.VideoWriter_fourcc('X', 'V', 'I', 'D')
out = cv.VideoWriter('./output.avi', fourcc, 20, (cap_width, cap_height))

print ('\n Video stream opened using OpenCV version: ' + cv.__version__ + '\n')

if not cap.isOpened():
    print("Cannot open camera")
    exit()

img = cv.imread("pic1.png", cv.IMREAD_COLOR)
hsv2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
#mask2 = cv.inRange(hsv2, lower_bound1, upper_bound1)
#mask2 = cv.inRange(mask2, lower_bound2, upper_bound2)
#mask2 = cv.morphologyEx(mask2, cv.MORPH_CLOSE, kernel)
#mask2 = cv.morphologyEx(mask2, cv.MORPH_OPEN, kernel)
cv.imshow("hello", img)

cv.waitKey(0)

exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    #gray = cv.cvtColor(frame, )
    # Display the resulting frame
    #flipped = cv.transpose(frame)
    #flipped = cv.flip(flipped, 0)
    #resiz = cv.resize(flipped, (cap_width, cap_height))
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_bound1, upper_bound1)
    mask = cv.inRange(hsv, lower_bound2, upper_bound2)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    out.write(hsv)
    cv.imshow('frame', mask)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
out.release()
cv.destroyAllWindows()
