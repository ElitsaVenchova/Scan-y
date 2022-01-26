"""
Structure light 3D scanner

@TODO:
    * Калибриране на камерата като се залепи големия лист със шахматна дъска на картона, на страната, която сега не се вижда.
        * да се направят няколк снимки с различна ориентация на дъската
    * Калибриране на проектор - да се прожектира шахматна дъска, която е принтирана и залепена на гърба. Да се засмене с камерата и да се калибрира проектора от нея.
    * Ново сканиране на обекта
        * Да се направи ОБРАТНО изкривяване на шаблона така, че като се прожектира, той да е правилен
        * Да се заснемат отново изображенията
        * Да се направи сканирането!
            * Има алгоритъм/ми за напрасване на една гледна точка към друга(за получаване на панорама)
            * са се прочете как се прави point cloud - .xyz(?)
            * да се прочете как от point cloud се прави point mesh - файловеи формати .ply, .stl, .obj
    * Да се оправи RuntimeWarning: This channel is already in use, continuing anyway.  Use GPIO.setwarnings(False) to disable warnings.
    * Web app
"""
import cv2 as cv
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
    scan.cameraCalibrate(chessboardSize, chessBlockSize, 'M')

    # patterns = sl.Patterns() # шаблони
    # patternsArr = patterns.genetare(patternCode,dsize) # шаблоните
    # for key, pattr in patternsArr.items():
    #     for i,img in enumerate(pattr):
    #         cv.imshow(key,img)
    #         cv.waitKey(0)
    #     cv.destroyAllWindows()

if __name__=="__main__":
    main()
