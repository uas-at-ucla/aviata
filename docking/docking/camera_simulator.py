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
PERIPHERAL_TARGET_SIZE=4.50
DRONE_RADIUS=1.1135 # Measured in meters, 1m boom + half the central tag side length (possibly slightly off for corners but good enough)

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
        self.scale_constant,self.periph_scale_constant = self.getViewScaleConstant(TARGET_SIZE, 1750)
        self.display_scale = 1750
        self.target_lat=target.getLat()
        self.target_lon=target.getLon()
        self.target_alt=target.getAlt()
        self.target_yaw=target.getYaw()       

        DISPLAY_SCALE = 1750 # Declares display factor constant (determines size of image relative to camera field of view)
        self.display_width = int(DISPLAY_SCALE * math.tan(math.radians(CAMERA_FOV_HORIZONTAL/2.0)))
        self.display_height = int(DISPLAY_SCALE * math.tan(math.radians(CAMERA_FOV_VERTICAL/2.0)))
        self.aspect_ratio = self.display_width / self.display_height

    # Calculates commonly used scale constant (to reduce runtime cost)
    def getViewScaleConstant(self, TARGET_SIZE, DISPLAY_SCALE_CONSTANT):
        return DISPLAY_SCALE_CONSTANT * TARGET_SIZE / 2.0 / 100.0, DISPLAY_SCALE_CONSTANT * PERIPHERAL_TARGET_SIZE / 2.0 / 100.0

    def updateCurrentImage(self, absLon, absLat, absAlt, absYaw,target=0):
        """
        Target faces north
        absLat -- drone's latitude in meters north
        absLon -- drone's longitude in meters east
        absAlt -- drone's altitude in meters up
        absYaw -- drone's yaw in degrees from north; positive is clockwise, negative is counterclockwise
                  -180 < absYaw < 180 (where 0 is true north and +/-180 is true south)
        target is id of docking target (0 for center, 1-8 counterclockwise for peripheral)
        """
        m1 = int(round(time.time() * 1000))

        # Converts from absolute to relative coordinates
        target_lat = self.target_lat
        target_lon = self.target_lon
        target_alt = self.target_alt
        target_yaw = self.target_yaw
        if target != 0: # Adjusts for offset peripheral targets
            target_offset = abs(target - 3) * 45 if target <= 3 else (11 - target) * 45
            target_lat += DRONE_RADIUS * math.sin(math.radians(target_offset - target_yaw))
            target_lon += DRONE_RADIUS * math.cos(math.radians(target_offset - target_yaw))
            target_yaw += -1 * 360 / 8 * (target - 1)

        relativeLat = absLat - target_lat
        relativeLon = absLon - target_lon
        relativeAlt = absAlt - target_alt
        relativeYaw = absYaw - target_yaw # degrees from north clockwise
        
        # Protects against invalid altitude
        if(relativeAlt <= 0):
            print("Altitude too low")
            return cv2.imread(self.background_image_name, 0)
            quit()

        # Resizes Image given the altitude and precomputed scale factor (scale precomputed for efficiency)
        if target==0:
            scale = abs(int(self.scale_constant / relativeAlt))
            april_tag = cv2.resize(self.april_tag, (scale, scale))
        else:
            tag_name="res/peripheral_tags/tag36_11_0000"+str(target)+".png"
            loaded_image=cv2.imread(tag_name) #Each image is 10x10,should be ok to load each iteration but could be preloaded
            scale = abs(int(self.periph_scale_constant / relativeAlt))
            april_tag = cv2.resize(loaded_image, (scale, scale),interpolation=cv2.INTER_NEAREST)

        # Sets translation of AprilTag
        # Converts cartesian translation to polar coordinates
        translationDist = math.sqrt(relativeLat**2 + relativeLon**2)
        translationAngle = np.arctan2(relativeLat, relativeLon) + math.radians(absYaw)

        # Adjusts for relative rotation then converts to pixel offset
        common_ops = self.scale_constant * translationDist / 2.0 / relativeAlt
        offsetx = math.cos(translationAngle) * common_ops
        offsety = math.sin(translationAngle) * common_ops

        # Rotates AprilTag without clipping
        # NOTE: opencv treats positive as ccw, but mavsdk treats positive as cw (which is what we use)
        # however, since the drone is rotating and not the image, the image must rotate in the opposite direction (so ccw)
        april_tag = self.rotate_image(april_tag, relativeYaw) 

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

        if x < background.shape[1] and y < background.shape[0]:
            background[y : y + img.shape[0], x : x + img.shape[1]] = img # paste tags on background
        cv2.imshow("Drone camera", background)
        cv2.waitKey(1)

        m2 = int(round(time.time() * 1000))
        return background


    def rotate_image(self, image, angle):
        """Modified from https://stackoverflow.com/questions/9041681/opencv-python-rotate-image-by-x-degrees-around-specific-point"""
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR, borderValue=(255, 255, 255))
        return result