#!/usr/bin/python3

from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video.pivideostream import PiVideoStream
import time
import cv2

FRAMERATE = 32
RESOLUTION = (720, 480)
stream = PiVideoStream(RESOLUTION, FRAMERATE).start()
result = cv2.VideoWriter('demo.avi', cv2.VideoWriter_fourcc(*'MJPG'), FRAMERATE, RESOLUTION)
time.sleep(1)
t = 0
tp = 0
nFrames = 0

# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
while True:
    # fps calculations
    t = time.time()
    nFrames += 1
    if t - tp >= 1:
        print(nFrames)
        nFrames = 0
        tp = t

    # image = frame.array
    image = stream.read()

    # cv2.imshow("Frame", image)
    # key = cv2.waitKey(1) & 0xFF

    result.write(image)
    # rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    # if key == ord("q"):
    #     break

result.release()