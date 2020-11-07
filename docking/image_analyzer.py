import cv2
import math
from apriltag import apriltag

TAG16 = "tag16h5" # tag family 
TAG36 = "tag36h11"
MIN_MARGIN = 10 # Filter value for tag detection
RED        = 0,0,255          # Colour of ident & frame (BGR)
detector = apriltag(TAG36)
CAM_HORIZ_FOV=62.2
TAG_SIZE=0.285 #Size of one side of tag in m

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
            det_corners=det["corners"].astype(int)
            det_center = det["center"].astype(int)

            rect = det["lb-rb-rt-lt"].astype(int).reshape((-1,1,2))
            cv2.polylines(img, [rect], True, RED, 2)
            pos = det_center + (-10,10)
            cv2.putText(img, "x", tuple(pos), cv2.FONT_HERSHEY_SIMPLEX, 1, RED, 2)
            cv2.putText(img, "c", img_center, cv2.FONT_HERSHEY_SIMPLEX, 1, RED, 2)

            #Calculates average side length
            sidelist=list()
            for i in range (0,4):
                for j in range (i,4):
                    sidelist.append(math.sqrt((det_corners[i][0]-det_corners[j][0])**2+(det_corners[i][1]-det_corners[j][1])))
            sidelist.sort()
            sideAvg=0
            for i in range(4):
                sideAvg=sideAvg+sidelist[i]
            sideAvg=sideAvg/4.0

            #Finds absolute difference in height
            alt_err=0.50/math.tan(CAMERA_HORIZ_FOV*0.50)*width*TAG_SIZE/sideAvg

            # raw errors
            x_offset = det_center[0] - img_center[0] # offset north from center
            y_offset = img_center[1] - det_center[1] # offset east from center

            # percentage errors
            x_err = x_offset / width
            y_err = y_offset / height

            return x_err, y_err, alt_err

            break

    cv2.imshow("TITLE", img)
    cv2.waitKey()

if __name__ == "__main__":
    # img = cv2.imread("apriltag_screen2.jpg")
    img = cv2.imread("sample-apriltag.jpg")

    scale_percent = 30 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    x_err, y_err = getErrors(resized)
    print(x_err, y_err)