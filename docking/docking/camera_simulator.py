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
        self.aspect_ratio = self.display_width / self.display_height

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
            return cv2.imread(self.background_image_name, 0)
            quit()

        # Scale factor for AprilTags given current altitude
        scale = abs(int(self.scale_constant / relativeAlt))

        # Sets translation of AprilTag (polar coordinates)
        translationDist = math.sqrt(relativeLat**2 + relativeLon**2)
        translationAngle = np.arctan2(relativeLon, relativeLat)
        common_ops = self.scale_constant * translationDist / 2.0 / relativeAlt # convert to pixel offset
        offsetx = math.cos(translationAngle) * common_ops
        offsety = math.sin(translationAngle) * common_ops

        # White canvas background for AprilTags
        background = cv2.imread(self.background_image_name)
        background = cv2.resize(background, (self.display_width, self.display_height))

        # Top left corner of upscaled AprilTag image
        x = int(self.display_width * 0.50 - offsetx - scale / 2.0)
        y = int(self.display_height * 0.50 + offsety - scale / 2.0)

        # As we descend, we need to upscale the AprilTag more and more. The edges are eventually going to overflow out
        # of our plain white background, so here we calculate the pixel offsets to crop
        x_min = y_min = 0
        x_max = y_max = scale
        sls = 4668 / scale # side length scale
        if y < 0: # crop top
            y_min = -y
            y = 0
        if x < 0: # crop left
            x_min = -x
            x = 0
        if scale + y > background.shape[1]: # crop bottom
            y_max = background.shape[1] - y + y_min
        if scale + x > background.shape[0]: # crop right
            x_max = background.shape[0] - x + x_min

        y_min = int(y_min * sls)
        x_min = int(x_min* sls)
        y_max = int(y_max * sls)
        x_max = int(x_max * sls)

        # At very low altitudes, we upscale the AprilTags so much we run out of memory. This solves the problem by cropping the
        # original AprilTag before upscaling it, so we only upscale the area of the image we need
        at = self.april_tag[y_min:y_max, x_min:x_max] # TODO: this turns the FOV into a square, losing our extended horizontal FOV

        at = cv2.resize(at, (0, 0), fx=(scale / 4668), fy=(scale / 4668))
        at = self.rotate_image(at, -1 * relativeYaw)
        m1 = int(round(time.time() * 1000))

        # Trim the edges in case of integer rounding error causing a few pixels of offset when scaling up
        if x + at.shape[1] > background.shape[1]: # at.shape[0] - (x + at.shape[0] - background.shape[0])
            at = at[0:at.shape[0], 0:background.shape[1]-x]
        if y + at.shape[0] > background.shape[0]:
            at = at[0:background.shape[0]-y, 0:at.shape[1]]
        background[y:y+at.shape[0], x:x+at.shape[1]] = at # paste tags on background
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
