import constants
import pygame
import numpy as np
import math
from PIL import Image
import cv2
import CameraSimulator

#Declares display factor constant (determines size of image relative to camera field of view)
DISPLAY_SCALE=1750

#Initializes display and image
pygame.init()
display_width=DISPLAY_SCALE*math.tan(math.radians(constants.CAMERA_FOV_HORIZONTAL/2.0))
display_height=DISPLAY_SCALE*math.tan(math.radians(constants.CAMERA_FOV_VERTICAL/2.0))
white=(255,255,255)
aprilTag=Image.open('APRILTAG.png')
displayWindow=pygame.display.set_mode((int(display_width),int(display_height)))
displayWindow.fill(white)

#Initializes scale constant
scale=CameraSimulator.getViewScaleConstant(constants.TARGET_SIZE,1750)
exited=False

#Main loop, can be used with different values if constantly updating
while not exited:
    #Gets simulated image given parameters, updates pygame window for debugging
    opencvImage=CameraSimulator.updateCurrentImage(aprilTag, scale, displayWindow, constants.RELATIVE_ALT,constants.RELATIVE_LAT,constants.RELATIVE_LON, constants.RELATIVE_YAW, DISPLAY_SCALE)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exited=True

pygame.quit()
quit()