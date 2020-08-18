import cv2
import numpy as np 
import re
import imutils
from os import listdir

files = listdir()
img_regex = re.compile('img/.*m\.jpg')

for file in files: 
    if not img_regex.match(file):
        continue

    img = cv2.imread(file)
    new_size = (int(img.shape[1] * .2), int(img.shape[0] * .2))
    img = cv2.resize(img, new_size)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


    # OpenCV uses 0-179 H, 0-255 S, 0-255 V
    # https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv
    lowerRange = np.array([2, 190, 130])
    upperRange = np.array([30, 255, 255])

    mask = cv2.inRange(hsv, lowerRange, upperRange)
    
    # identify shape
    blurred = cv2.GaussianBlur(mask, (7, 7), 0)
    edges = cv2.Canny(blurred, 100, 500)

    cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)
        # ensure that the approximated contour is "roughly" rectangular
        print(len(approx))
        if len(approx) >= 4 and len(approx) <= 6:
            print("YES")
    





    cv2.imshow(file, edges)
    cv2.waitKey()

    # cv2.imwrite("mask_" + file, mask)