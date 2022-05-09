import numpy as np
import cv2 as cv
import time
cap = cv.VideoCapture(0)

cap.set(cv.CAP_PROP_FRAME_WIDTH, 1080)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1920)

cap_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
cap_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

print(cap_width)
print(cap_height)

fps_start_time = 0
fps = 0

fourcc = cv.VideoWriter_fourcc('X', 'V', 'I', 'D')
out = cv.VideoWriter('./output.avi', fourcc, 20, (cap_width, cap_height))

print ('\n Video stream opened using OpenCV version: ' + cv.__version__ + '\n')
if not cap.isOpened():
    print("Cannot open camera")
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
    flipped = cv.transpose(frame)
    flipped = cv.flip(flipped, 0)
    resiz = cv.resize(flipped, (cap_width, cap_height))
    out.write(resiz)
    cv.imshow('frame', flipped)
    
    # FPS calculation
    fps_end_time = time.time()
    time_diff = fps_end_time - fps_start_time
    fps = 1/(time_diff)
    fps_start_time = fps_end_time
    
    fps_text = "FPS: {:.2f}".format(fps)
    
    cv2.putText(frame, fps_text, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 100), 1)
    
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
out.release()
cv.destroyAllWindows()
