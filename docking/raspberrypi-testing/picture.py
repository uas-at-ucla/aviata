#! /usr/bin/python3

from picamera.array import PiRGBArray
from picamera import PiCamera

import numpy as np

import cv2
import time

# import re
# from os import listdir

camera = PiCamera()
rawCapture = PiRGBArray(camera)

time.sleep(1)

camera.capture(rawCapture, format="bgr")
image = rawCapture.array

cv2.imshow("Image", image)
cv2.waitKey(0)