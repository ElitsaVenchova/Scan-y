"""
Structure light 3D scanner

@TODO:
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
    cSize =(1920,1080) # размер на камерата
    pSize =(615,360) #8,4 # размер на проектора
    chessboardSize = (8,6)
    chessBlockSize = 16 # mm

    scan = sl.StructuredLight(cSize, pSize)
    #scan.scan(patternCode)
    scan.cameraCalibrate(chessboardSize, chessBlockSize)

    # patterns = sl.Patterns() # шаблони
    # patternsArr = patterns.genetare(patternCode,dsize) # шаблоните
    # for key, pattr in patternsArr.items():
    #     for i,img in enumerate(pattr):
    #         cv2.imshow(key,img)
    #         cv2.waitKey(0)
    #     cv2.destroyAllWindows()

if __name__=="__main__":
    main()
