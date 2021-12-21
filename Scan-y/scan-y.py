"""
Structure light 3D scanner

@TODO:
    * На калибрирането да се добави условие, ако има шаблон, тогава да прави снимка
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
    dsize =(820,480) #640,480#8,4
    chessboardSize = (8,6)

    scan = sl.StructuredLight(dsize, chessboardSize)
    scan.scan(patternCode)
    #scan.cameraCalibrate()

    # patterns = sl.Patterns() # шаблони
    # patternsArr = patterns.genetare(patternCode,dsize) # шаблоните
    # for key, pattr in patternsArr.items():
    #     for i,img in enumerate(pattr):
    #         cv2.imshow(key,img)
    #         cv2.waitKey(0)
    #     cv2.destroyAllWindows()

if __name__=="__main__":
    main()
