import cv2
from imutils.video.pivideostream import PiVideoStream
import imutils
import time
import numpy as np

class VideoCamera(object):
    def __init__(self):
        self.vs = PiVideoStream().start()
        time.sleep(2.0)

    def __del__(self):
        self.vs.stop()

    def get_frame(self):
        frame = self.vs.read()
##        np.flip(frame, 0) ##да се откоментира, ако е необходимо обръщане
        ret, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()
