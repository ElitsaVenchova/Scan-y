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

    # Инициализиране на необходимите параметри за сканиране и калибриране
    def __init__(self):
        self.turntable = Turntable() # въртящата се маса
        self.cameraPi = CameraPi() # камера
        self.patterns = Patterns() # шаблони
        self.projector = Projector(self.cameraPi) # проектор

    # Сканиране на 360*.
    # На всяка стъпка се прави снимка без шаблон и снимка с всеки шаблон
    # stepSize - размер на стъпката. stepSize=1 е завъртане на 1.8 градуса. stepSize=200 е 200*1.8=3600 градуса.
    def scan(self, patternCode, stepSize, theshold):
        reconstruct3D = Reconstruct3D(self.cameraPi) # инициализиране на обект Реконструиране
        # шаблоните
        patternImgs = self.patterns.genetare(patternCode,self.cameraPi.stereoCalibrationRes['pShape']) # шаблоните
        calibRes = self.cameraPi.getUndistortCalibrationRes(self.SCAN_DIR) # резултатите от калибрането необходими за сканиране(в SCAN_DIR)

        # self.projector.start()
        # # интериране позициите на масата за завъртане на 360*
        # for i in range(0, self.turntable.SPR, stepSize):
        #     for pattType, patt in patternImgs.items():
        #         self.scanCurrentStep(patt, self.SCAN_DIR, pattType, i, calibRes)
        #     self.turntable.step(stepSize)
        # self.projector.stop()

        reconstruct3D.reconstruct(self.SCAN_DIR, patternCode, self.turntable.SPR, stepSize, theshold) # реконструиране на резултата от сканирането.

    # Стерео калибриране на проектора и камерата едновременно. Тук се генерира и информацията за тяхното разположение спрямо сцената.
    def stereoCalibrate(self, chessboardSize):
        patternCode = Patterns.CHESS_BOARD
        patternImgs = self.patterns.genetare(patternCode,self.projector.pCalibrationRes["shape"],None,chessboardSize) # генериране на шаблон Шахматна дъска

        self.projector.start() # заснемане на всички генерирани шаблони
        for pattType, patt in patternImgs.items():
            self.scanCurrentStep(patt, self.cameraPi.STEREO_CALIBRATION_DIR, pattType, 0)
        self.projector.stop()
        self.cameraPi.stereoCalibrate(chessboardSize, self.projector.pCalibrationRes) # стерео калибране

    # Калибриране на камерата.
    # type: A-автоматичен,M-ръчен
    # calibImgCnt - колко изображения да се заснемат. Използва се при ръчното сканиране
    def cameraCalibrate(self, pSize, chessboardSize, chessBlockSize, calibType='A',calibImgCnt=None):
        # бял шаблон
        patternCode = Patterns.WHITE
        patternImgs = self.patterns.genetare(patternCode,pSize) # генериране само на бял шаблон
        pattType, patt = list(patternImgs.items())[0]

        self.projector.start() # Заснемане на изображения за калибриране според подадения calibType
        if calibType == 'M':
            self.manualCameraCalibrate(patternImgs,pattType, patt, calibImgCnt)
        else:
            self.autoCameraCalibrate(patternImgs,pattType, patt)

        self.projector.stop()
        self.cameraPi.calibrate(self.cameraPi.CALIBRATION_DIR, chessboardSize, chessBlockSize) # калибриране на камерата

    def manualCameraCalibrate(self, patternImgs,pattType, patt, calibImgCnt):
        # Засменане на calibImgCnt изображения като на всяка стъпка се оставя време за нагласяне
        # заснема се изображение при натискане на някое копче на клавиатурата.
        for i in range(0, calibImgCnt):
            input('Fix image and press <<Enter>>!')
            self.scanCurrentStep(patt, self.cameraPi.CALIBRATION_DIR, pattType, i)

    def autoCameraCalibrate(self, patternImgs,pattType, patt, calibImgCnt):
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
        patternImgs = self.patterns.genetare(patternCode,pSize,chessboardSize) # генериране на шаблон Шахматна дъска
        pattType, patt = list(patternImgs.items())[0] #Взимане само на първия шаблон. Тук ръчно се върти и премества проектора.

        # Засменане на calibImgCnt изображения като на всяка стъпка се оставя време за нагласяне
        # заснема се изображение при натискане на някое копче на клавиатурата.
        self.projector.start()
        for i in range(0, calibImgCnt):
            input('Fix image and press <<Enter>>!')
            self.scanCurrentStep(patt, self.projector.CALIBRATION_DIR, pattType, i)

        self.projector.stop()
        self.cameraPi.calibrate(self.projector.CALIBRATION_DIR,chessboardSize, 1, pSize) # Калибриране на проектора. Не може да се определи големината на шахматния квадрат и се подава <1>

    def scanCurrentStep(self, patternImgs, dir, patternName, stepNo, pCalibrationRes=None):
        # итериране по шаблоните като enumerate добави пореден номер за улеснение
        for i,img in enumerate(patternImgs):
            # Ако има налична калибрация на проетора и извикване за калибриране на проектора,
            # то първо се "изправя" изображението преди да се прожектира
            if pCalibrationRes != None:
                img = self.cameraPi.undistortImage(img,pCalibrationRes)
            # cv.imshow('image',img)
            self.projector.playImage(img)
            time.sleep(2)
            self.cameraPi.takePhoto(dir,'{0}{1}{2}'.format(stepNo,patternName,i))
