import cv2
import numpy as np 
import re
from os import listdir

files = listdir()
img_regex = re.compile('dots.jpg')

for file in files: 
    if not img_regex.match(file):
        continue

    img = cv2.imread(file)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lowerRange = np.array([0, 170, 100])
    upperRange = np.array([100, 255, 255])

    mask = cv2.inRange(hsv, lowerRange, upperRange)

    width = int(mask.shape[1] * .2)
    height = int(mask.shape[0] * .2)

    cv2.imwrite("mask_" + file, mask)