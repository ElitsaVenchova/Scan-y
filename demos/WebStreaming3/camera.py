import cv2
import picamera
import time
import numpy as np
from threading import Condition
import io

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)
    
class VideoCamera(object):
    def __init__(self):
        self.camera = picamera.PiCamera(resolution='640x480', framerate=24)
        self.output = StreamingOutput()
        self.camera.start_recording(self.output, format='mjpeg')


    def __del__(self):
        self.camera.stop_recording()      
        time.sleep(2.0)

    def get_frame(self):
       frame = self.output.frame
##        np.flip(frame, 0) ##да се откоментира, ако е необходимо обръщане
##        ret, jpeg = cv2.imencode('.jpg', frame)
       return frame
