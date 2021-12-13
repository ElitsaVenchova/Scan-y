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
        # Размер на екрана/прожекцията
        self.dsize = dsize
        # въртящата се маса
        self.turntable = Turntable()
        # камерата
        self.piCamera = CameraPi(chessboardSize)
        # шаблоните
        self.patterns = Patterns()

    # Сканиране на 360*.
    # На всяка стъпка се прави снимка без шаблон и снимка с всеки шаблон
    def scan(self, patternCode):
        # шаблоните
        patternImgs = self.patterns.genetare(patternCode,self.dsize) # шаблоните
        patternImgsTran = self.patterns.transpose(patternImgs) # шаблоните транспонирани
        patternImgsInv = self.patterns.invert(patternImgs) # шаблоните обърнати(ч->б,б->ч)
        patternImgsInvTran = self.patterns.invert(patternImgsInv) # шаблоните обърнати(ч->б,б->ч) и транспонирани

        # интериране позициите на масата за завъртане на 360*
        for i in range(self.turntable.SPR):
            self.scanCurrentStep(patternImgs, "Img", i)
            self.scanCurrentStep(patternImgsTran, "ImgTran", i)
            self.scanCurrentStep(patternImgsInv, "ImgInv", i)
            self.scanCurrentStep(patternImgsInvTran, "ImgInvTran", i)
            self.turntable.step()

    def scanCurrentStep(self, patternImgs, patternName, stepNo):
        ind = 0
        for img in patternImgs:
            cv2.imshow('image',img)
            self.piCamera.takePhoto(self.SCAN_DIR,"{0}{1}{2}".format(stepNo,patternName,ind))
            ind += 1
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    def cameraCalibrate(self):
        # бял шаблон
        patternCode = Patterns.WHITE
        patternImgs = self.patterns.genetare(patternCode,self.dsize) # шаблоните
        # интериране позициите на масата за завъртане на 360*
        for i in range(self.turntable.SPR):
            cv2.imshow('image',patternImgs[0])
            self.piCamera.takePhoto(self.piCamera.CALIBRATION_DIR,i)
            cv2.waitKey(1)
            self.turntable.step()
        self.piCamera.calibrate()
