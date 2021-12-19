import cv2
import numpy as np
import math

from .patterns import Patterns
from .cameraPi import CameraPi
from .turntable import Turntable

"""
    Освен клас съдржащ всички методи за калибриране, сканиране и обработване на данните
"""
class StructuredLight:

    SCAN_DIR = "Scan" # Директория съдържаща снимките за сканирането

    # Задаване на OUT pin-овете и размер на стъпката
    def __init__(self, dsize, chessboardSize):
        self.dsize = dsize # Размер на екрана/прожекцията
        self.turntable = Turntable() # въртящата се маса
        self.piCamera = CameraPi(chessboardSize) # камера
        self.patterns = Patterns() # шаблони

    # Сканиране на 360*.
    # На всяка стъпка се прави снимка без шаблон и снимка с всеки шаблон
    def scan(self, patternCode):
        # шаблоните
        patternImgs = self.patterns.genetare(patternCode,self.dsize) # шаблоните

        # интериране позициите на масата за завъртане на 360*
        for i in range(self.turntable.SPR):
            for pattType, patt in thisdict.items():
                self.scanCurrentStep(patt, self.SCAN_DIR, pattType, i)
                self.turntable.step()

    def cameraCalibrate(self):
        # бял шаблон
        patternCode = Patterns.WHITE
        patternImgs = self.patterns.genetare(patternCode,self.dsize) # шаблоните

        # интериране позициите на масата за завъртане на 360*
        for i in range(self.turntable.SPR):
            for pattType, patt in thisdict.items():
                self.scanCurrentStep(patt, self.piCamera.CALIBRATION_DIR, pattType, i)
                self.turntable.step()
        self.piCamera.calibrate()

    def scanCurrentStep(self, patternImgs, dir, patternName, stepNo):
    # итериране по шаблоните като enumerate добави пореден номер за улеснение
    for i,img in enumerate(patternImgs):
        # cv2.imshow('image',img)
        self.piCamera.takePhoto(dir,"{0}{1}{2}".format(stepNo,patternName,i))
        cv2.waitKey(1)
    cv2.destroyAllWindows()
