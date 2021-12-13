from time import sleep
import RPi.GPIO as GPIO

"""
    Контролиране на масата чрез въртене на стъпковия мотор
"""
class Turntable:

    STEP = 20  # Step GPIO Pin
    DIR = 21   # Direction GPIO Pin
    # pin-ове за определяне на размера на стъпката, която се подава
    MS1 = 25 # GPIO Pin
    MS2 = 8 # GPIO Pin
    MS3 = 7 # GPIO Pin
    # Константи на стъпковия мотор
    CW = 1     # Clockwise Rotation
    CCW = 0    # Counterclockwise Rotation
    SPR = 200   # Стъпки на завъртане (360 / 1.8)
    DELAY = .004

    # Задаване на OUT pin-овете и размер на стъпката
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(DIR, GPIO.OUT)
        GPIO.setup(STEP, GPIO.OUT)
        GPIO.setup(MS1, GPIO.OUT)
        GPIO.setup(MS2, GPIO.OUT)
        GPIO.setup(MS3, GPIO.OUT)
        self.setStepSize();

    # MS1   MS2  MS3   Microstep Resolution
    #------------------------------------
    # Low   Low  Low   Full step
    # High  Low  Low   Half step
    # Low   High Low   Quarter step
    # High  High Low   Eighth step
    # High  High High  Sixteenth step
    def setStepSize(self):
        #Sixteenth step
        GPIO.output(MS1, GPIO.HIGH)
        GPIO.output(MS2, GPIO.HIGH)
        GPIO.output(MS3, GPIO.HIGH)

    # Прави cnt стъпки на мотора. Default 1(*16, защото се подава 1/16 стъпка в метод setStepSize())
    def step(self, dir = CW, cnt = 1*16):
        GPIO.output(DIR, dir)
        for x in range(cnt):
            GPIO.output(STEP, GPIO.HIGH)
            sleep(DELAY)
            GPIO.output(STEP, GPIO.LOW)
            sleep(DELAY)
