#! /usr/bin/python3

from picamera.array import PiRGBArray
from picamera import PiCamera

import numpy as np

import cv2
import time

### import re
### from os import listdir

# initialize camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# camera warmup
time.sleep(1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array # image as array
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0) # clear stream

    if key == ord("q"):
        break
