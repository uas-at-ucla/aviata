import constants
import pygame
import numpy as np
import math
from PIL import Image

#Calculates commonly used scale constant (to reduce runtime cost)
def getViewScaleConstant(TARGET_SIZE,DISPLAY_SCALE_CONSTANT):
    return DISPLAY_SCALE_CONSTANT*TARGET_SIZE/2.0/100.0

def updateCurrentImage(aprilTag, scaleConstant, display, relativeAlt,relativeLat,relativeLon, relativeYaw, display_scale):
    #Protects against invalid altitude
    if(constants.RELATIVE_ALT<=0):
        print("Altitude too low")
        pygame.quit()
        quit()
    
    #Resizes Image given the altitude and precomputed scale factor(scale precomputed for efficiency)
    scale=int(scaleConstant/relativeAlt)
    aprilTag=aprilTag.resize((scale,scale))

    #Sets translation of AprilTag
    #Converts cartesian translation to polar coordinates
    translationDist=math.sqrt(relativeLat**2+relativeLon**2)
    translationAngle=math.degrees(np.arctan2(relativeLat,relativeLon))
    #Adjusts for relative rotation then converts to pixel offset
    offsetx=display_scale*translationDist*math.cos(math.radians(translationAngle-relativeYaw))/2.0/relativeAlt
    offsety=display_scale*translationDist*math.sin(math.radians(translationAngle-relativeYaw))/2.0/relativeAlt

    #Rotates AprilTag
    aprilTag=aprilTag.rotate(-1*relativeYaw)
    #Displays image
    aprilTag.save("updatedTag.png")
    aprilTag=pygame.image.load("updatedTag.png")
    display.blit(aprilTag, (display.get_width()*0.50-offsetx-scale/2.0,display.get_height()*0.50+offsety-scale/2.0))
    pygame.display.update()

