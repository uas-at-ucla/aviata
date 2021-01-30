from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2
import time

RESOLUTION = (640, 480)
ROTATION = 0

class RaspiCamera:
    def __init__(self, resolution=(640, 480), framerate=32):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
            format="bgr", use_video_port=True)
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)
            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def read(self):
        # return the frame most recently read
        ret = self.frame
        self.frame = [] # only read each frame once
        return ret

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

if __name__ == "__main__":
    camera = RaspiCamera()
    camera.start()
    time.sleep(2) # warm up
    while True:
        m1 = int(round(time.time() * 1000))
        img = camera.read()
        while img == []:
            img = camera.read()
        m2 = int(round(time.time() * 1000))
        print(m2 - m1)
        
    camera.stop()
    cv2.imwrite("test.png", img)