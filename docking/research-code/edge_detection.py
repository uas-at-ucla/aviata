import cv2
import numpy as np 
import imutils
import re
from os import listdir

files = listdir()
img_regex = re.compile('img/postit_tape.*\.jpg')

for file in files: 
    if not img_regex.match(file):
        continue

    img = cv2.imread(file)
    img = cv2.resize(img, (int(img.shape[1] * .5), int(img.shape[0] * .5)))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # extract portion of image that matches desired color
    lowerRange = np.array([12, 80, 160])
    upperRange = np.array([38, 255, 255])
    mask = cv2.inRange(hsv, lowerRange, upperRange)

    # identify rectangular shape
    # https://www.pyimagesearch.com/2015/05/04/target-acquired-finding-targets-in-drone-and-quadcopter-video-streams-using-python-and-opencv/
    blurred = cv2.GaussianBlur(mask, (7, 7), 0)
    edges = cv2.Canny(blurred, 50, 150)
    cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)

        # ensure that the approximated contour is "roughly" rectangular
        area = cv2.contourArea(c)
        hullArea = cv2.contourArea(cv2.convexHull(c))
        if hullArea == 0:
            continue
        solidity = area / float(hullArea)
        print(solidity)
        print(len(approx))
        if solidity > 0.97 and len(approx) <= 8:
            cv2.drawContours(img, [approx], -1, (0, 0, 255), 1)

    cv2.imwrite("rec_" + file, img)