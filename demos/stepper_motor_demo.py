from time import sleep
import RPi.GPIO as GPIO

STEP = 20  # Step GPIO Pin
DIR = 21   # Direction GPIO Pin

# MS1   MS2  MS3   Microstep Resolution
#------------------------------------
# Low   Low  Low   Full step
# High  Low  Low   Half step
# Low   High Low   Quarter step
# High  High Low   Eighth step
# High  High High  Sixteenth step
MS1 = 25
MS2 = 8
MS3 = 7

CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = 200*16   # Steps per Revolution (360 / 7.5)


GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

#Sixteenth step
GPIO.setup(MS1, GPIO.OUT)
GPIO.setup(MS2, GPIO.OUT)
GPIO.setup(MS3, GPIO.OUT)
GPIO.output(MS1, GPIO.HIGH)
GPIO.output(MS2, GPIO.HIGH)
GPIO.output(MS3, GPIO.HIGH)

GPIO.output(DIR, CW)

step_count = SPR
delay = .004

for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)

sleep(.5)
GPIO.output(DIR, CCW)
for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)
