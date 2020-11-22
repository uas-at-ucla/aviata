import cv2
import math
from apriltag import apriltag

TAG16 = "tag16h5" # tag family 
TAG36 = "tag36h11"
MIN_MARGIN = 10 # Filter value for tag detection
RED        = 0,0,255          # Colour of ident & frame (BGR)
detector = apriltag(TAG36)
CAMERA_HORIZ_FOV=62.2
CAMERA_VERTICAL_FOV=48.8
TAG_SIZE=0.285 #Size of one side of tag in m
CAMERA_FOCUS_X=0.034
CAMERA_FOCUS_Y=0.034

def getErrors(img):
    """
    Returns the error for an image 
    """

    # detect
    greys = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    dets = detector.detect(greys)
    if len(dets) == 0:
        print("No apriltag detected, aborting")
        return
    elif len(dets) > 1:
        print("WARNING: More than 1 apriltag detected, will use the first one")
    

    # get image dimensions
    height, width, _ = img.shape
    img_center = (int(width / 2), int(height / 2))

    for det in dets:
        if det["margin"] >= MIN_MARGIN:
            det_center = det["center"].astype(int)

            # Draws corners and rectangle
            rect = det["lb-rb-rt-lt"].astype(int).reshape((-1,1,2))
            cv2.polylines(img, [rect], True, RED, 2)
            pos = det_center + (-10,10)
            cv2.putText(img, "x", tuple(pos), cv2.FONT_HERSHEY_SIMPLEX, 1, RED, 2)
            cv2.putText(img, "c", img_center, cv2.FONT_HERSHEY_SIMPLEX, 1, RED, 2)

            # Calculates average side length 
            sidelist = list()
            for i in range (0,4):
                for j in range (i+1,4):
                    sidelist.append(math.sqrt((rect[i][0][0] - rect[j][0][0]) ** 2 + (rect[i][0][1] - rect[j][0][1]) ** 2))
                    # print(sidelist)
            sidelist.sort()
            sideAvg = 0
            for i in range(4):
                sideAvg = sideAvg+sidelist[i]
            sideAvg = sideAvg/4.0
            tag_pixel_ratio = TAG_SIZE / sideAvg

            # Absolute difference in height (meters)
            alt_err = 0.50 / math.tan(math.radians(CAMERA_HORIZ_FOV * 0.50)) * width * tag_pixel_ratio

            # Finds difference in rotation
            # rot_err=math.atan2(0.50*(rect[2][0][1]+rect[3][0][1]-rect[0][0][1]-rect[1][0][1]),0.50*(rect[2][0][0]-rect[3][0][0]+rect[1][0][0]-rect[0][0][0]))
            y3 = rect[3][0][1]
            y0 = rect[0][0][1]
            x3 = rect[3][0][0]
            x0 = rect[0][0][0]

            # account for small errors
            if abs(y3 - y0) < 3:
                y3 = y0
            if abs(x3 - x0) < 3:
                x3 = x0

            print(rect)
            if x3 - x0 != 0:
                incline_angle = math.degrees(math.atan2(y3 - y0, x3 - x0))
            else:
                incline_angle = 0
            
            rot_err = 0
            if y3 <= y0 and x3 > x0: # y's are flipped since pixel coordinates has origin at top left corner instead of bottom left
                rot_err = 90 - incline_angle
            elif y3 > y0 and x3 >= x0:
                rot_err = 180 - incline_angle
            elif y3 >= y0 and x3 < x0:
                rot_err = 270 - incline_angle
            else: # y3 < y0 and x3 <= x0:
                rot_err = 360 - incline_angle
            
            print(rot_err)

            # Absolute horizontal difference (meters)
            x_offset = det_center[0] - img_center[0] # offset east from center
            y_offset = img_center[1] - det_center[1] # offset north from center
            x_err = x_offset * tag_pixel_ratio
            y_err = y_offset * tag_pixel_ratio

            print("Errors: ", x_err, " ", y_err, " ", alt_err, " ", rot_err)
            return float(x_err), float(y_err), float(alt_err), float(rot_err)

            break

    cv2.imshow("TITLE", img)
    cv2.waitKey()

if __name__ == "__main__":
    # img = cv2.imread("apriltag_screen2.jpg")
    img = cv2.imread("updatedTag.png")

    scale_percent = 30 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    x_err, y_err, z_err, rot_err = getErrors(resized)
    print(x_err, y_err, z_err, rot_err)
