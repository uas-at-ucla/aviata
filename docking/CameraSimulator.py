import pygame
import numpy as np
import math
import cv2
from PIL import Image

#################
## Copy of Camera\ Simulator/CameraSimulator.py to get around weird import errors
## Modifications to make it into a class, set target initial position
#################

#Relative position and rotation of drone
#Measured in meters, East and North are positive
RELATIVE_LAT=0.0
RELATIVE_LON=0.0
RELATIVE_ALT=22.5
#Measured degrees counterclockwise from North
RELATIVE_YAW=45.0

#Camera information
#Measured in degrees
CAMERA_FOV_VERTICAL=48.8
CAMERA_FOV_HORIZONTAL=62.2

#Target information
#Measured in centimeters
TARGET_SIZE=28.5

class CameraSimulator:

    background_image_name = 'Camera Simulator/BACKGROUND.jpg'
    output_tag_name = 'updatedTag.png'
    april_tag = Image.open('Camera Simulator/APRILTAG2.png')

    def __init__(self, initial_lat, initial_long):
        # self.initial_alt = initial_alt
        self.initial_lat = initial_lat
        self.initial_long = initial_long
        # self.initial_yaw = initial_yaw
        self.scale_constant = self.getViewScaleConstant(TARGET_SIZE, 1750)
        self.display_scale = 1750

        # Declares display factor constant (determines size of image relative to camera field of view)
        DISPLAY_SCALE = 1750

        # Initializes display and image
        pygame.init()
        display_width = DISPLAY_SCALE * \
            math.tan(math.radians(CAMERA_FOV_HORIZONTAL/2.0))
        display_height = DISPLAY_SCALE * \
            math.tan(math.radians(CAMERA_FOV_VERTICAL/2.0))
        white = (255, 255, 255)
        aprilTag = Image.open('Camera Simulator/APRILTAG2.png')
        self.display = pygame.display.set_mode(
            (int(display_width), int(display_height)))
        self.display.fill(white)

    # Calculates commonly used scale constant (to reduce runtime cost)
    def getViewScaleConstant(self, TARGET_SIZE, DISPLAY_SCALE_CONSTANT):
        return DISPLAY_SCALE_CONSTANT*TARGET_SIZE/2.0/100.0

    def updateCurrentImage(self, relativeAlt, relativeLat, relativeLon, relativeYaw):
        # relativeAlt += self.initial_alt
        relativeLat += self.initial_lat
        relativeLon += self.initial_long
        # relativeYaw += self.initial_yaw

        # Protects against invalid altitude
        if(RELATIVE_ALT <= 0):
            print("Altitude too low")
            return cv2.imread(self.background_image_name, 0)
            pygame.quit()
            quit()

        # Resizes Image given the altitude and precomputed scale factor(scale precomputed for efficiency)
        scale = int(self.scale_constant/relativeAlt)
        april_tag = self.april_tag.resize((scale, scale))

        # Sets translation of AprilTag
        # Converts cartesian translation to polar coordinates
        translationDist = math.sqrt(relativeLat**2+relativeLon**2)
        translationAngle = math.degrees(np.arctan2(relativeLat, relativeLon))
        # Adjusts for relative rotation then converts to pixel offset
        offsetx = self.scale_constant*translationDist * \
            math.cos(math.radians(translationAngle-relativeYaw)) / \
            2.0/relativeAlt
        offsety = self.scale_constant*translationDist * \
            math.sin(math.radians(translationAngle-relativeYaw)) / \
            2.0/relativeAlt

        # Rotates AprilTag without clipping
        april_tag = april_tag.rotate(-1*relativeYaw, expand=True)

        # Displays image for debug
        april_tag.save(self.output_tag_name)
        white = (255, 255, 255)
        self.display.fill(white)
        self.display.blit(pygame.image.load(self.output_tag_name), (self.display.get_width()*0.50-offsetx -
                                           scale/2.0, self.display.get_height()*0.50+offsety-scale/2.0))

        # Converts image to OpenCV format and returns it
        background = Image.open(self.background_image_name)
        img = Image.open(self.output_tag_name)
        background = background.resize(
            (self.display.get_width(), self.display.get_height()), Image.ANTIALIAS)
        background.paste(img, (int(self.display.get_width()*0.50-offsetx -
                                   scale/2.0), int(self.display.get_height()*0.50+offsety-scale/2.0)))
        opencvImage = cv2.cvtColor(np.array(background), cv2.COLOR_RGB2BGR)
        pygame.display.update()
        return opencvImage



if __name__ == "__main__":
    cs = CameraSimulator(4, -3)

    while True:
        cs.updateCurrentImage(5, 0, 0, 0)
