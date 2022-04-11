import cv2 as cv
import numpy as np
import math
import os
import time

from .patterns import Patterns
from .cameraPi import CameraPi
from .turntable import Turntable
from .projector import Projector
from .reconstruct3D import Reconstruct3D

"""
    Освен клас съдържащ всички методи за калибриране, сканиране и обработване на данните
"""
class StructuredLight:

    SCAN_DIR = "./Scan" # Директория съдържаща снимките за сканирането
    # Определя големината на една стъпка.
    STEP_SIZE = 10 # 200/1 - ще се сканира от 200 ъгъла; 200/10=20 - ще се сканира от 20 ъгъла

    # Инициализиране на необходимите параметри за сканиране и калибриране
    def __init__(self):
        self.turntable = Turntable() # въртящата се маса
        self.cameraPi = CameraPi() # камера
        self.patterns = Patterns() # шаблони
        self.projector = Projector(self.cameraPi) # проектор

    # Сканиране на 360*.
    # На всяка стъпка се прави снимка без шаблон и снимка с всеки шаблон
    def scan(self, patternCode):
        reconstruct3D = Reconstruct3D(self.cameraPi) # реконструиране на обекта
        # шаблоните
        patternImgs = self.patterns.genetare(patternCode,self.cameraPi.stereoCalibrationRes['pShape']) # шаблоните
        calibRes = self.cameraPi.getUndistortCalibrationRes(self.SCAN_DIR)

        self.projector.start()
        # интериране позициите на масата за завъртане на 360*
        for i in range(0, self.turntable.SPR, self.STEP_SIZE):
            for pattType, patt in patternImgs.items():
                self.scanCurrentStep(patt, self.SCAN_DIR, pattType, i, calibRes)
            self.turntable.step(self.STEP_SIZE)
        self.projector.stop()

        reconstruct3D.reconstruct(self.SCAN_DIR, patternCode)

    def stereoCalibrate(self, chessboardSize):
        patternCode = Patterns.CHESS_BOARD
        patternImgs = self.patterns.genetare(patternCode,self.projector.pCalibrationRes["shape"],chessboardSize) # генериране само на бял шаблон

        self.projector.start()
        for pattType, patt in patternImgs.items():
            self.scanCurrentStep(patt, self.cameraPi.STEREO_CALIBRATION_DIR, pattType, 0)
        self.projector.stop()
        self.cameraPi.stereoCalibrate(chessboardSize, self.projector.pCalibrationRes)


    # Калибриране на камерата.
    # type: A-автоматичен,M-ръчен
    def cameraCalibrate(self, pSize, chessboardSize, chessBlockSize, calibType='A'):
        # бял шаблон
        patternCode = Patterns.WHITE
        patternImgs = self.patterns.genetare(patternCode,pSize) # генериране само на бял шаблон
        pattType, patt = list(patternImgs.items())[0]

        self.projector.start()
        if calibType == 'M':
            self.manualCameraCalibrate(patternImgs,pattType, patt)
        else:
            self.autoCameraCalibrate(patternImgs,pattType, patt)

        self.projector.stop()
        self.cameraPi.calibrate(self.cameraPi.CALIBRATION_DIR, chessboardSize, chessBlockSize)

    def manualCameraCalibrate(self, patternImgs,pattType, patt, calibImgCnt):
        for i in range(0, calibImgCnt):
            input('Fix image and press <<Enter>>!')
            self.scanCurrentStep(patt, self.cameraPi.CALIBRATION_DIR, pattType, i)

    def autoCameraCalibrate(self, patternImgs,pattType, patt):
        # местим шахматната дъска 20 позии наляво и 20 надясно
        # резултатът е 40 изображения за калибриране
        for i in range(0, 20):
            self.scanCurrentStep(patt, self.cameraPi.CALIBRATION_DIR, pattType, i)
            self.turntable.step(1, self.turntable.CW)# стъпка по часовниковата стрелка
        self.turntable.step(20, self.turntable.CCW) # връщане в изходна позиция
        for i in range(20, 40):
            self.scanCurrentStep(patt, self.cameraPi.CALIBRATION_DIR, pattType, i)
            self.turntable.step(1, self.turntable.CCW)# стъпка обратно по часовниковата стрелка
        self.turntable.step(19, self.turntable.CW) # връщане в изходна позиция

    # Калибриране на проектора
    def projectorCalibrate(self, pSize, chessboardSize, calibImgCnt):
        patternCode = Patterns.CHESS_BOARD
        patternImgs = self.patterns.genetare(patternCode,pSize,chessboardSize) # генериране само на бял шаблон
        pattType, patt = list(patternImgs.items())[0]

        self.projector.start()
        for i in range(0, calibImgCnt):
            input('Fix image and press <<Enter>>!')
            self.scanCurrentStep(patt, self.projector.CALIBRATION_DIR, pattType, i)

        self.projector.stop()
        self.cameraPi.calibrate(self.projector.CALIBRATION_DIR,chessboardSize, 1, pSize) #Не може да се определи големината на шахматния квадрат

    def scanCurrentStep(self, patternImgs, dir, patternName, stepNo, pCalibrationRes=None):
        # итериране по шаблоните като enumerate добави пореден номер за улеснение
        for i,img in enumerate(patternImgs):
            # Ако има налична калибрация на проетора и извикването не за калибриране на проектора,
            # то първо се "изправя изображението" преди да се прожектира
            if pCalibrationRes != None:
                img = self.cameraPi.undistortImage(img,pCalibrationRes)
            # cv.imshow('image',img)
            self.projector.playImage(img)
            time.sleep(2)
            self.cameraPi.takePhoto(dir,'{0}{1}{2}'.format(stepNo,patternName,i))
