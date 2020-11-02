import constants
import pygame
import numpy as np
import math
import cv2
from PIL import Image

#Calculates commonly used scale constant (to reduce runtime cost)
def getViewScaleConstant(TARGET_SIZE,DISPLAY_SCALE_CONSTANT):
    return DISPLAY_SCALE_CONSTANT*TARGET_SIZE/2.0/100.0

def updateCurrentImage(aprilTag, scaleConstant, display, relativeAlt,relativeLat,relativeLon, relativeYaw, display_scale):
    #Protects against invalid altitude
    if(constants.RELATIVE_ALT<=0):
        print("Altitude too low")
        return cv2.imread('BACKGROUND.jpg',0)
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

    #Rotates AprilTag without clipping
    aprilTag=aprilTag.rotate(-1*relativeYaw,expand=True)

    #Displays image for debug
    aprilTag.save("updatedTag.png")
    aprilTag=pygame.image.load("updatedTag.png")
    white=(255,255,255)
    display.fill(white)
    display.blit(aprilTag, (display.get_width()*0.50-offsetx-scale/2.0,display.get_height()*0.50+offsety-scale/2.0))
    
    #Converts image to OpenCV format and returns it
    background=Image.open("BACKGROUND.jpg")
    img=Image.open("updatedTag.png")
    background=background.resize((display.get_width(),display.get_height()),Image.ANTIALIAS)
    background.paste(img,(int(display.get_width()*0.50-offsetx-scale/2.0),int(display.get_height()*0.50+offsety-scale/2.0)))
    opencvImage=cv2.cvtColor(np.array(background),cv2.COLOR_RGB2BGR)
    pygame.display.update()
    return opencvImage

