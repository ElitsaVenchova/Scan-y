#Camera demo
from picamera import PiCamera
from time import sleep

camera=PiCamera()
try:
    #camera.rotation = 180
    camera.start_preview(fullscreen=False,window=(50,80,1000,1200))#alpha=230,
    sleep(5)
#--------------Photo
    camera.capture('image.jpg')
#--------------Video
##    camera.start_recording('/home/pi/Desktop/Diplomna/video.h264')
##    sleep(10)
##    camera.stop_recording()
finally:
    camera.stop_preview()
    camera.close()
