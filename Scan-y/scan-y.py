"""
Structure light 3D scanner

@TODO:
    * Калибриране на камерата
    * Проектиране на проектор
    * Калибриране на проектор
    * Обработка на сканираните изображения
"""
import cv2
import numpy as np
import structuredlight as sl
import math

def main():
    width, height = 640,480#640,480#8,4
    patternCode = sl.Patterns.WHITE
    scan(patternCode,(width, height))

# Сканиране на 360*.
# На всяка стъпка се прави снимка без шаблон и снимка с всеки шаблон
def scan(patternCode,dsize):
    # въртящата се маса
    turntable = sl.Turntable()
    # камерата
    piCamera = sl.CameraPi()
    # шаблоните
    patterns = sl.Patterns()
    patternImgs = patterns.genetare(patternCode,dsize) # шаблоните
    patternImgsTran = patterns.transpose(patternImgs) # шаблоните транспонирани
    patternImgsInv = patterns.invert(patternImgs) # шаблоните обърнати(ч->б,б->ч)
    patternImgsInvTran = patterns.invert(patternImgsInv) # шаблоните обърнати(ч->б,б->ч) и транспонирани

    # интериране позициите на масата за завъртане на 360*
    for i in range(turntable.SPR):
        scanCurrentStep(piCamera, patternImgs, "Img", i)
        scanCurrentStep(piCamera, patternImgsTran, "ImgTran", i)
        scanCurrentStep(piCamera, patternImgsInv, "ImgInv", i)
        scanCurrentStep(piCamera, patternImgsInvTran, "ImgInvTran", i)
        turntable.step()

def scanCurrentStep(piCamera, patternImgs, patternName, stepNo):
    imageName = str(stepNo)+patternName
    ind = 0
    for img in patternImgs:
        cv2.imshow('image',img)
        piCamera.takePhoto(imageName +str(ind))
        ind += 1
        cv2.waitKey(0)
        # cv2.waitKey(1)
    cv2.destroyAllWindows()

def cameraCalibrate():
    pass


if __name__=="__main__":
    main()
