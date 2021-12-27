import cv2
import numpy as np
import math

from .patterns import Patterns
from .cameraPi import CameraPi
from .turntable import Turntable
from .projector import Projector

"""
    Освен клас съдржащ всички методи за калибриране, сканиране и обработване на данните
"""
class StructuredLight:

    SCAN_DIR = "Scan" # Директория съдържаща снимките за сканирането
    # Определя големината на една стъпка.
    STEP_SIZE = 25 # 200/1 - ще се сканира от 200 ъгъла; 200/25=8 - ще се сканира от 8 ъгъла

    # Инициализиране на необходимите параметри за сканиране и калибриране
    def __init__(self, cSize, pSize):
        self.cSize = cSize # Резолюциата на камерата
        self.pSize = pSize # Размер прожекцията
        self.turntable = Turntable() # въртящата се маса
        self.piCamera = CameraPi() # камера
        self.patterns = Patterns() # шаблони
        self.projector = Projector() # проектор

    # Сканиране на 360*.
    # На всяка стъпка се прави снимка без шаблон и снимка с всеки шаблон
    def scan(self, patternCode):
        # шаблоните
        patternImgs = self.patterns.genetare(patternCode,self.pSize) # шаблоните

        self.projector.start()
        # интериране позициите на масата за завъртане на 360*
        for i in range(0, self.turntable.SPR, self.STEP_SIZE):
            for pattType, patt in patternImgs.items():
                self.scanCurrentStep(patt, self.SCAN_DIR, pattType, i)
            self.turntable.step(self.STEP_SIZE)
        self.projector.stop()

    def cameraCalibrate(self, chessboardSize, chessBlockSize):
        # бял шаблон
        patternCode = Patterns.WHITE
        patternImgs = self.patterns.genetare(patternCode,self.pSize) # шаблоните

        self.projector.start()
        # местим шахматната дъска 20 позии наляво и 20 надясно
        # резултатът е 40 изображения за калибриране
        for i in range(0, 20):
            for pattType, patt in patternImgs.items():
                self.scanCurrentStep(patt, self.piCamera.CALIBRATION_DIR, pattType, i)
            self.turntable.step(1, self.turntable.CW)# стъпка по часовниковата стрелка
        self.turntable.step(20, self.turntable.CCW) # връщане в изходна позиция
        for i in range(20, 40):
            for pattType, patt in patternImgs.items():
                self.scanCurrentStep(patt, self.piCamera.CALIBRATION_DIR, pattType, i)
            self.turntable.step(1, self.turntable.CCW)# стъпка обратно по часовниковата стрелка
        self.turntable.step(19, self.turntable.CW) # връщане в изходна позиция
        
        self.piCamera.calibrate(chessboardSize, chessBlockSize)
        self.projector.stop()

    def scanCurrentStep(self, patternImgs, dir, patternName, stepNo):
        # итериране по шаблоните като enumerate добави пореден номер за улеснение
        for i,img in enumerate(patternImgs):
            # cv2.imshow('image',img)
            self.projector.playImage(img)
            self.piCamera.takePhoto(dir,"{0}{1}{2}".format(stepNo,patternName,i))
        cv2.destroyAllWindows()
