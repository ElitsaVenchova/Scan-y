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
    patternCode = sl.Patterns.GRAY_CODE
    dsize =(640,480) #640,480#8,4
    chessboardSize = (8,6)

    scan = sl.StructuredLight(dsize, chessboardSize)
    # scan.scan(patternCode)
    scan.cameraCalibrate()

    ### @TODO: Debug patterns
    # patternImgs = scan.genetare(patternCode,(width, height)) # шаблоните
    # for img in patternImgs:
    #     cv2.imshow('image',img)
    #     cv2.waitKey(0)
    # cv2.destroyAllWindows()

if __name__=="__main__":
    main()
