import pygame
import numpy as np
import math
import cv2
import time

# Relative position and rotation of drone

# Measured in meters, East and North are positive
RELATIVE_LAT=0.0
RELATIVE_LON=0.0
RELATIVE_ALT=22.5

# Measured degrees counterclockwise from North
RELATIVE_YAW=45.0

# Camera information
# Measured in degrees
CAMERA_FOV_VERTICAL=48.8
CAMERA_FOV_HORIZONTAL=62.2

# Target information
# Measured in centimeters
TARGET_SIZE=234

class CameraSimulator:

    background_image_name = 'res/BACKGROUND.jpg'
    april_tag = cv2.imread('res/3_stage_tags.png')

    # Updates target location dynamically
    def updateTargetLocation(self, target):
        self.target_lat=target.getLat()
        self.target_lon=target.getLon()
        self.target_alt=target.getAlt()
        self.target_yaw=target.getYaw()

    def __init__(self,target):
        self.scale_constant = self.getViewScaleConstant(TARGET_SIZE, 1750)
        self.display_scale = 1750
        self.target_lat=target.getLat()
        self.target_lon=target.getLon()
        self.target_alt=target.getAlt()
        self.target_yaw=target.getYaw()       

        DISPLAY_SCALE = 1750 # Declares display factor constant (determines size of image relative to camera field of view)
        self.display_width = int(DISPLAY_SCALE * math.tan(math.radians(CAMERA_FOV_HORIZONTAL/2.0)))
        self.display_height = int(DISPLAY_SCALE * math.tan(math.radians(CAMERA_FOV_VERTICAL/2.0)))

    # Calculates commonly used scale constant (to reduce runtime cost)
    def getViewScaleConstant(self, TARGET_SIZE, DISPLAY_SCALE_CONSTANT):
        return DISPLAY_SCALE_CONSTANT * TARGET_SIZE / 2.0 / 100.0

    def updateCurrentImage(self, absLat, absLon, absAlt, absYaw):
        """
        Target faces north
        absLat -- drone's latitude in meters north
        absLon -- drone's longitude in meters east
        absAlt -- drone's altitude in meters up
        absYaw -- drone's yaw in degrees from north; positive is clockwise, negative is counterclockwise
                  -180 < absYaw < 180 (where 0 is true north and +/-180 is true south)
        """
        startm = int(round(time.time() * 1000))

        # Converts from absolute to relative coordinates
        relativeLat = absLat - self.target_lat
        relativeLon = absLon - self.target_lon
        relativeAlt = absAlt - self.target_alt
        relativeYaw = absYaw - self.target_yaw # degrees from north clockwise

        # Protects against invalid altitude
        if(relativeAlt <= 0):
            print("Altitude too low")
            pygame.quit()
            return cv2.imread(self.background_image_name, 0)
            quit()

        # Resizes Image given the altitude and precomputed scale factor (scale precomputed for efficiency)
        scale = abs(int(self.scale_constant / relativeAlt))
        april_tag = cv2.resize(self.april_tag, (scale, scale))

        # Sets translation of AprilTag
        # Converts cartesian translation to polar coordinates
        translationDist = math.sqrt(relativeLat**2 + relativeLon**2)
        translationAngle = np.arctan2(relativeLon, relativeLat)

        # Adjusts for relative rotation then converts to pixel offset
        common_ops = self.scale_constant * translationDist / 2.0 / relativeAlt
        offsetx = math.cos(translationAngle) * common_ops
        offsety = math.sin(translationAngle) * common_ops

        # Rotates AprilTag without clipping
        april_tag = self.rotate_image(april_tag, -1 * relativeYaw)
        m1 = int(round(time.time() * 1000))

        # Puts AprilTags on white canvas background, clipping edges if necessary
        background = cv2.imread(self.background_image_name)
        background = cv2.resize(background, (self.display_width, self.display_height))
        img = april_tag

        x = int(self.display_width * 0.50 - offsetx - scale / 2.0)
        y = int(self.display_height * 0.50 + offsety - scale / 2.0)
        if y < 0: # crop top
            img = img[-y:img.shape[0], 0:img.shape[1]]
            y = 0
        if x < 0: # crop left
            img = img[0:img.shape[0], -x:img.shape[1]]
            x = 0
        if img.shape[0] + y > background.shape[0]: # crop bottom
            img = img[0:background.shape[0] - y, 0:img.shape[1]]
        if img.shape[1] + x > background.shape[1]: # crop right
            img = img[0:img.shape[0], 0:background.shape[1] - x]

        background[y : y + img.shape[0], x : x + img.shape[1]] = img # paste tags on background
        cv2.imshow("Drone camera", background)
        cv2.waitKey(1)

        m2 = int(round(time.time() * 1000))
        # print("Calc time: ", m1 - startm, " render time: ", m2 - m1)
        return background

    def rotate_image(self, image, angle):
        """Modified from https://stackoverflow.com/questions/9041681/opencv-python-rotate-image-by-x-degrees-around-specific-point"""
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR, borderValue=(255, 255, 255))
        return result
