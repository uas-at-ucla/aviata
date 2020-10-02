#! /usr/bin/python3

from picamera.array import PiRGBArray
from picamera import PiCamera

import numpy as np

import cv2
import time
import imutils

# initialize camera
# camera = PiCamera()
# print(camera.resolution)
# rawCapture = PiRGBArray(camera, size=(1280, 720))
# time.sleep(1)
camera = PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 60
camera.exposure_mode = 'off'
camera.shutter_speed = 20000
rawCapture = PiRGBArray(camera, size=(1280, 720))

result = cv2.VideoWriter('demo.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (1280, 720))
nFrames = 0

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array # image as array

    # camera.capture(rawCapture, format="bgr")
    # img = rawCapture.array
    # cv2.imshow("Image", img)
    # cv2.waitKey(0)

    # convert to grayscale, blur, detect edges
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    edged = cv2.Canny(blurred, 50, 150)
    # cv2.imshow("edges", edged)
    # cv2.waitKey(0)

    # find contours from edges
    contours = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    for c in contours:
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)

        # ensure that the approximated contour is "roughly" rectangular
        area = cv2.contourArea(c)
        hullArea = cv2.contourArea(cv2.convexHull(c))
        if hullArea == 0:
            continue
        solidity = area / float(hullArea)
        if solidity > 0.90 and len(approx) <= 6:
            # find midpoint of image vs midpoint of target, determine where to move based on these two points
            sum_x = 0
            sum_y = 0
            for row in approx:
                for subrow in row:
                    sum_x += subrow[0]
                    sum_y += subrow[1]
            avg_x = sum_x / 4
            avg_y = sum_y / 4 # may need to change divisor based on # contours later, possible inaccuracies

            print(avg_x, avg_y)
            print(avg_x - 640, avg_y - 640)
            if abs(avg_x - 640) > 1280 * .05 and abs(avg_y - 360) > 720 * .05:
                cv2.drawContours(img, [approx], -1, (0, 0, 255), 1)
                cv2.arrowedLine(img, (640, 360), (int(avg_x), int(avg_y)), (0, 0, 255), 2)
            else: 
                cv2.drawContours(img, [approx], -1, (0, 255, 0), 1)
            break

    result.write(img)
    rawCapture.truncate(0) # clear stream
    
    nFrames += 1
    if nFrames == 100:
        break

result.release()
cv2.destroyAllWindows()