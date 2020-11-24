import cv2
from apriltag import apriltag
 
TITLE      = "apriltag_view"  # Window title
TAG        = "tag36h11"        # Tag family
MIN_MARGIN = 10               # Filter value for tag detection
FONT       = cv2.FONT_HERSHEY_SIMPLEX  # Font for ID value
RED        = 0,0,255          # Colour of ident & frame (BGR)
 
fname = 'updatedTag.png' # name of the image to detect Apriltags in
img = cv2.imread(fname)

detector = apriltag(TAG)
greys = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
dets = detector.detect(greys)
for det in dets:
    if det["margin"] >= MIN_MARGIN:
        rect = det["lb-rb-rt-lt"].astype(int).reshape((-1,1,2))
        cv2.polylines(img, [rect], True, RED, 2)
        ident = str(det["id"])
        pos = det["center"].astype(int) + (-10,10)
        cv2.putText(img, ident, tuple(pos), FONT, 1, RED, 2)
cv2.imshow("TITLE", img)
cv2.waitKey()