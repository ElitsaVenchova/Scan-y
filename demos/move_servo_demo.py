#Move servo demo
from gpiozero import Servo
from time import sleep
import sys

servo=Servo(20)
val=-1
#------v1
##while True:
##    servo.value=val
##    sleep(0.1)
##    val=val+0.1
##    if val>1:
##        val=-1
#------v2
#    servo.min()
#    sleep(1)
#    servo.max()
#------v3
servo.value=1
sleep(2)
servo.value=0
sleep(2)
servo.value=1
#
#sys.exit()
quit()
