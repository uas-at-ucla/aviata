import pygame
import numpy as np
import math
import cv2
from PIL import Image
from image_analyzer import getErrors

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
    april_tag = Image.open('Camera Simulator/tag_36h11.png')

    #Updates target location dynamically
    def updateTargetLocation(self, targetx,targety,targetalt,targetyaw):
        self.target_lat=targetx
        self.target_lon=targety
        self.target_alt=targetalt
        self.target_yaw=targetyaw

    def __init__(self,targetx,targety,targetalt,targetyaw):
        self.scale_constant = self.getViewScaleConstant(TARGET_SIZE, 1750)
        self.display_scale = 1750
        self.target_lat=targetx
        self.target_lon=targety
        self.target_alt=targetalt
        self.target_yaw=targetyaw        # Declares display factor constant (determines size of image relative to camera field of view)
        DISPLAY_SCALE = 1750

        # Initializes display and image
        pygame.init()
        display_width = DISPLAY_SCALE * math.tan(math.radians(CAMERA_FOV_HORIZONTAL/2.0))
        display_height = DISPLAY_SCALE * math.tan(math.radians(CAMERA_FOV_VERTICAL/2.0))
        white = (255, 255, 255)
        aprilTag = Image.open('Camera Simulator/tag_36h11.png')
        self.display = pygame.display.set_mode((int(display_width), int(display_height)))
        self.display.fill(white)

    # Calculates commonly used scale constant (to reduce runtime cost)
    def getViewScaleConstant(self, TARGET_SIZE, DISPLAY_SCALE_CONSTANT):
        return DISPLAY_SCALE_CONSTANT*TARGET_SIZE/2.0/100.0

    def updateCurrentImage(self, absLat, absLon, absAlt, absYaw):
        """
        Target faces north
        absLat -- drone's latitude in meters north
        absLon -- drone's longitude in meters east
        absAlt -- drone's altitude in meters up
        absYaw -- drone's yaw in degrees from north; positive is clockwise, negative is counterclockwise
                  -180 < absYaw < 180 (where 0 is true north and +/-180 is true south)
        """
        #Converts from absolute to relative coordinates
        relativeLat=absLat-self.target_lat
        relativeLon=absLon-self.target_lon
        relativeAlt=absAlt-self.target_alt
        relativeYaw=absYaw-self.target_yaw # degrees from north clockwise

        # Protects against invalid altitude
        if(relativeAlt <= 0):
            print("Altitude too low")
            pygame.quit()
            return cv2.imread(self.background_image_name, 0)
            quit()

        # Resizes Image given the altitude and precomputed scale factor (scale precomputed for efficiency)
        scale = abs(int(self.scale_constant/relativeAlt))
        april_tag = self.april_tag.resize((scale, scale))

        # Sets translation of AprilTag
        # Converts cartesian translation to polar coordinates
        translationDist = math.sqrt(relativeLat**2+relativeLon**2)
        translationAngle = math.degrees(np.arctan2(relativeLon, relativeLat))
        # Adjusts for relative rotation then converts to pixel offset
        offsetx = self.scale_constant*translationDist * \
            math.cos(math.radians(translationAngle)) / \
            2.0/relativeAlt
        offsety = self.scale_constant*translationDist * \
            math.sin(math.radians(translationAngle)) / \
            2.0/relativeAlt

        # Rotates AprilTag without clipping
        april_tag = april_tag.rotate(-1*relativeYaw, expand=True, fillcolor="#ffffff")

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
    cs = CameraSimulator(0, 0, 5, 0)

    angle = 0
    while True:
       image = cs.updateCurrentImage(0, 0, 8, angle)
       angle = angle + 1
       errors = getErrors(image)
       print(errors)
