from picamera.array import PiRGBArray
from picamera import PiCamera
import time

RESOLUTION = (640, 480)
ROTATION = 0

class RaspiCamera:
    def __init__(self):
        self.camera = PiCamera()
        self.camera_array = PiRGBArray(self.camera)
        self.camera.resolution = RESOLUTION
        self.camera.rotation = ROTATION
        time.sleep(1)
    
    def updateCurrentImage(self):
        self.camera.capture(self.camera_array, format="bgr")
        image = self.camera_array.array
        self.camera_array.truncate(0)
        return image 

    def closeCamera(self):
        self.camera.close()