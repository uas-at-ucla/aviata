import cv2
import math
import time
import numpy as np
from apriltag import apriltag

TAG16               = "tag16h5" # tag family 
TAG36               = "tag36h11"
MIN_MARGIN          = 10 # Filter value for tag detection
detector            = apriltag(TAG36)
CAMERA_HORIZ_FOV    = 62.2
CAMERA_VERTICAL_FOV = 48.8
CENTRAL_TAG_SIZE    = 0.227 # size of one side of tag in m
PERIPHERAL_TAG_SIZE = 0.045

class ImageAnalyzer:

    def process_image(self, img, ind, yaw):
        """
        Process an image to determine how far off the drone is
        Return the x, y, and z offset in meters, yaw offset in degrees clockwise
        ind tells simulator which tag to target (0 for center, 1-8 counterclockwise from North for perimeter)
        yaw is the drone's current rotation, used for rectifying the rotation back to north before processing
        """

        # get image dimensions
        height, width, _ = img.shape
        img_center = (int(width / 2), int(height / 2))

        # rotate to due north, so we can use north/east/down (ned) coordinates instead of body coordinates
        image_center = tuple(np.array(img.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, yaw * -1, 1.0)
        result = cv2.warpAffine(img, rot_mat, img.shape[1::-1], flags=cv2.INTER_LINEAR, borderValue=(255, 255, 255))
        img = result

        # detect
        greys = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        dets = detector.detect(greys)
        
        tags_detected=""
        for det in dets:
            if(det["margin"] >= MIN_MARGIN):
                tags_detected += str(det["id"]) + " "

        for det in dets:
            if det["id"] == ind:
                if det["margin"] >= MIN_MARGIN:
                    det_center = det["center"].astype(int)

                    # Extract apriltag bounding box
                    rect = det["lb-rb-rt-lt"].astype(int).reshape((-1,1,2))

                    # Calculates average side length in pixels
                    sidelist = list()
                    for i in range (0,4):
                        for j in range (i+1,4):
                            sidelist.append(math.sqrt((rect[i][0][0] - rect[j][0][0]) ** 2 + (rect[i][0][1] - rect[j][0][1]) ** 2))
                    sidelist.sort()
                    sideAvg = 0
                    for i in range(4):
                        sideAvg = sideAvg+sidelist[i]
                    sideAvg = sideAvg/4.0
                    tag_pixel_ratio = (CENTRAL_TAG_SIZE if ind == 0 else PERIPHERAL_TAG_SIZE) / sideAvg # ratio of meters to pixels

                    # Absolute difference in height (meters)
                    alt_err = 0.50 / math.tan(math.radians(CAMERA_HORIZ_FOV * 0.50)) * width * tag_pixel_ratio

                    # Finds difference in rotation
                    y3 = rect[3][0][1]
                    y0 = rect[0][0][1]
                    x3 = rect[3][0][0]
                    x0 = rect[0][0][0]

                    if x3 - x0 != 0:
                        # Look at bottom left corner and top left corner to determine angle from y-axis
                        incline_angle = math.degrees(math.atan2(y3 - y0, x3 - x0))

                        # Convert angle so that 0 degrees is north, positive angles are clockwise, negative angles are ccw
                        # -180 < incline_angle < 180
                        incline_angle = incline_angle * -1 + 90
                        if incline_angle > 0:
                            incline_angle = incline_angle - 180
                        else:
                            incline_angle = incline_angle + 180
                    elif y3 > y0:
                        incline_angle = -180
                    else:
                        incline_angle = 0
                    rot_err = incline_angle * -1 # to make this cw

                    # Absolute horizontal difference (meters)
                    x_offset = det_center[0] - img_center[0] # offset east from center
                    y_offset = img_center[1] - det_center[1] # offset north from center
                    x_err = x_offset * tag_pixel_ratio
                    y_err = y_offset * tag_pixel_ratio

                    # cv2.polylines(img, [rect], True, (0, 0, 255), 2)
                    # cv2.imshow("image analyzed", img)
                    # cv2.waitKey(1)

                    return float(x_err), float(y_err), float(alt_err), float(rot_err), tags_detected

                    break

        print("Desired apriltag not found, aborting")
        return

        # cv2.imshow("TITLE", img)
        # cv2.waitKey()

# if __name__ == "__main__":
#     img = cv2.imread("/home/axel/Pictures/a5.png")

#     scale_percent = 30 # percent of original size
#     width = int(img.shape[1] * scale_percent / 100)
#     height = int(img.shape[0] * scale_percent / 100)
#     dim = (width, height)
#     # resize image
#     resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

#     image_analyzer = ImageAnalyzer()
#     x_err, y_err, z_err, rot_err, e = image_analyzer.process_image(resized, 1)
