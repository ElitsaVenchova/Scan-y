"""
Structure light 3D scanner

@TODO:
    * Проектиране на проектор
    * Калибриране на проектор
    * Обработка на сканираните изображения
    * Web app
"""
import cv2
import numpy as np
import structuredlight as sl
import math

def main():
    patternCode = sl.Patterns.GRAY_CODE
    dsize =(640,480) #640,480#8,4
    chessboardSize = (8,6)

    scan = sl.StructuredLight(dsize, chessboardSize)
    scan.scan(patternCode)
    #scan.cameraCalibrate()

if __name__=="__main__":
    main()
