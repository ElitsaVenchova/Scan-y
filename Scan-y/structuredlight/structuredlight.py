import cv2 as cv
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
    def __init__(self, pSize):
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

    # Калибриране на камерата.
    # type: A-автоматичен,M-ръчен
    def cameraCalibrate(self, chessboardSize, chessBlockSize, calibType='A'):
        # бял шаблон
        patternCode = Patterns.WHITE
        patternImgs = self.patterns.genetare(patternCode,self.pSize) # генериране само на бял шаблон
        pattType, patt = list(patternImgs.items())[0]

        self.projector.start()
        if calibType == 'M':
            self.manualCameraCalibrate(patternImgs,pattType, patt)
        else:
            self.autoCameraCalibrate(patternImgs,pattType, patt)

        self.projector.stop()
        self.piCamera.calibrate(self.piCamera.CALIBRATION_DIR, chessboardSize, chessBlockSize)

    def manualCameraCalibrate(self, patternImgs,pattType, patt):
        pattType, patt = list(patternImgs.items())[0]
        for i in range(0, 20):
            input('Fix image and press <<Enter>>!')
            self.scanCurrentStep(patt, self.piCamera.CALIBRATION_DIR, pattType, i)

    def autoCameraCalibrate(self, patternImgs,pattType, patt):
        # местим шахматната дъска 20 позии наляво и 20 надясно
        # резултатът е 40 изображения за калибриране
        for i in range(0, 20):
            self.scanCurrentStep(patt, self.piCamera.CALIBRATION_DIR, pattType, i)
            self.turntable.step(1, self.turntable.CW)# стъпка по часовниковата стрелка
        self.turntable.step(20, self.turntable.CCW) # връщане в изходна позиция
        for i in range(20, 40):
            self.scanCurrentStep(patt, self.piCamera.CALIBRATION_DIR, pattType, i)
            self.turntable.step(1, self.turntable.CCW)# стъпка обратно по часовниковата стрелка
        self.turntable.step(19, self.turntable.CW) # връщане в изходна позиция

    # Калибриране на проектора
    def projectorCalibrate(self, chessboardSize):
        patternCode = Patterns.CHESS_BOARD
        patternImgs = self.patterns.genetare(patternCode,self.pSize,chessboardSize) # генериране само на бял шаблон
        pattType, patt = list(patternImgs.items())[0]

        self.projector.start()
        for i in range(0, 20):
            input('Fix image and press <<Enter>>!')
            self.scanCurrentStep(patt, self.projector.CALIBRATION_DIR, pattType, i)

        self.projector.stop()
        self.piCamera.calibrate(self.projector.CALIBRATION_DIR,chessboardSize, 1) #Не може да се определи големината на шахматния квадрат

    def scanCurrentStep(self, patternImgs, dir, patternName, stepNo):
        print(stepNo)
        # итериране по шаблоните като enumerate добави пореден номер за улеснение
        for i,img in enumerate(patternImgs):
            # cv.imshow('image',img)
            self.projector.playImage(img)
            self.piCamera.takePhoto(dir,"{0}{1}{2}".format(stepNo,patternName,i))
        cv.destroyAllWindows()
