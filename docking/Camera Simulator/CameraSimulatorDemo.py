import constants
import pygame
import numpy as np
import math
from PIL import Image

#Initializes display
pygame.init()
display_width=1750.0*math.tan(math.radians(constants.CAMERA_FOV_HORIZONTAL/2.0))
display_height=1750.0*math.tan(math.radians(constants.CAMERA_FOV_VERTICAL/2.0))
white=(255,255,255)
aprilTag=Image.open('APRILTAG.png')
displayWindow=pygame.display.set_mode((int(display_width),int(display_height)))
displayWindow.fill(white)

#Check for valid altitude, can be changed with display output if needed for PX4 integration
if(constants.RELATIVE_ALT<=0):
    pygame.quit()
    quit()

#Calculates height and width of field of view of camera
horizCameraView=2.0*constants.RELATIVE_ALT*math.tan(math.radians(constants.CAMERA_FOV_HORIZONTAL/2.0))
vertCameraView=2.0*constants.RELATIVE_ALT*math.tan(math.radians(constants.CAMERA_FOV_VERTICAL/2.0))
#Resizes AprilTag image proportional to field of view of camera
horizScale=display_width*constants.TARGET_SIZE/100.0/horizCameraView
vertScale=display_height*constants.TARGET_SIZE/100.0/vertCameraView
aprilTag=aprilTag.resize((int(horizScale),int(vertScale)))

#Sets translation of AprilTag
#Converts cartesian translation to polar coordinates
translationDist=math.sqrt(constants.RELATIVE_LAT*constants.RELATIVE_LAT+constants.RELATIVE_LON*constants.RELATIVE_LON)
translationAngle=math.degrees(np.arctan2(constants.RELATIVE_LAT,constants.RELATIVE_LON))
#Adjusts for relative rotation then converts to pixel offset
offsetx=display_width*translationDist*math.cos(math.radians(translationAngle-constants.RELATIVE_YAW))/horizCameraView
offsety=display_height*translationDist*math.sin(math.radians(translationAngle-constants.RELATIVE_YAW))/vertCameraView

#Rotates AprilTag
aprilTag=aprilTag.rotate(-1*constants.RELATIVE_YAW)

#Displays image
aprilTag.save("updatedTag.png")
aprilTag=pygame.image.load("updatedTag.png")
displayWindow.blit(aprilTag, (display_width*0.50-offsetx-horizScale/2.0,display_height*0.50+offsety-vertScale/2.0))
pygame.display.update()

#Keeps window open
exited=False
while not exited:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exited=True

pygame.quit()
quit()

