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
    * Пример как да приложа резултатите от калибрирането
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
