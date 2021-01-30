from apriltag import apriltag
from imutils.video.pivideostream import PiVideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera

import cv2
import time

TITLE      = "apriltag_view"  # Window title

# these need to be changed depending on tag/environment choice
TAG_FAMILY = "tag36h11"
TAG        = 0                # which tag in the family we're looking for
#TAG_SIZE   = 

MIN_MARGIN = 10               # Filter value for tag detection
FONT       = cv2.FONT_HERSHEY_SIMPLEX
RED        = 0,0,255
GREEN      = 0,255,0

ALLOWED_ERROR = 0.05          # 5% error is allowed

# FRAMERATE = 32
# RESOLUTION = (720, 480)
 
if __name__ == '__main__':

    detector = apriltag(TAG_FAMILY)
    stream = cv2.VideoCapture(0)

    width = stream.get(3)
    height = stream.get(4)
    print(width, height)

    # stream = PiVideoStream(RESOLUTION, FRAMERATE).start()
    # time.sleep(1) # let camera warm up

    while cv2.waitKey(1) != 0x1b:
        _, img = stream.read()
        greys = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        dets = detector.detect(greys)
        for det in dets:
            if det["margin"] >= MIN_MARGIN and det['id'] == TAG:
                rect = det["lb-rb-rt-lt"].astype(int).reshape((-1,1,2))
                pos = det["center"].astype(int)

                ok_to_dock = True

                # check centered
                width_error = abs(pos[0] - width / 2)
                height_error = abs(pos[1] - height / 2)
                if width_error > width * ALLOWED_ERROR and height_error > height * ALLOWED_ERROR:
                    ok_to_dock = False
                    box_color = RED
                    start = (int(pos[0]), int(pos[1]))
                    end = (int(width / 2), int(width / 2))
                    cv2.arrowedLine(img, start, end, RED, 2)

                # check height

                # check rotation

                cv2.polylines(img, [rect], True, RED, 2)

                # cv2.putText(img, ident, tuple(pos), FONT, 1, RED, 2)
        cv2.imshow(TITLE, img)
    cv2.destroyAllWindows()
