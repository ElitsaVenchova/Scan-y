"""
Structure light 3D scanner

@TODO:
    * Въртене на маса
    * Проектиране на проектор
    * Калибриране на камера
    * Калибриране на проектор
    * !!!СКАНИРАНЕ!!! = проектиране, въртене меса, заснемане, обработка
"""
import cv2
import numpy as np
import structuredlight as sl
import math

def main():
    width, height = 640,480#640,480#8,4
    showPatterns((width, height))

def showPatterns(dsize):
    patterns = sl.Patterns()
    gray = patterns.phaseShifting(dsize)
    # gray = patterns.transpose(gray)
    for img in gray :
        cv2.imshow('image',img )
        cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()
