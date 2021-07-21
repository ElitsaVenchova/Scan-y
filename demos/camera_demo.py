#Camera demo
from picamera import PiCamera
from time import sleep

camera=PiCamera()
try:
    #camera.start_preview()
    #sleep(5)
#--------------Photo
    #camera.capture('/home/pi/Desktop/Diplomna/image.jpg')
#--------------Video
    camera.start_recording('/home/pi/Desktop/Diplomna/video.h264')
    sleep(10)
    camera.stop_recording()
finally:
    #camera.stop_preview()
    camera.close()
