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
# import structuredlight as sl
import math
import argparse

def main(args):
    pSize =(615,360) #размер на проектора
    chessboardSize = (8,6)
    chessBlockSize = 16 # mm

    scan = sl.StructuredLight(pSize)
    if args.action == 'S':
        patternCode = args.pattern
        scan.scan(patternCode)
    elif args.action == 'CC':
        scan.cameraCalibrate(chessboardSize, chessBlockSize, args.calib_type)
    elif args.action == 'PC':
        scan.projectorCalibrate(chessboardSize)
    else:
        raise ValueError('Bad action code!')

if __name__=="__main__":
    parser = argparse.ArgumentParser(
        description='Scan-y 3D structured light scanner\n',
        formatter_class=argparse.RawTextHelpFormatter)
    # nargs='?' - позволява парсването на параметрите, ако нищо не е подадено
    parser.add_argument('action', type=str, nargs='?',help='What action will be performed? S-scanning, CC-camera calibration, PC-projector calibration')
    parser.add_argument('pattern', type=int, nargs='?',help='Pattern for scanning.'
                                '0-WHITE, 1-BINARY, 2-STRIPE, 3-GRAY_CODE, 4-PHASE_SHIFTING, 5(default)-GRAY_CODE_AND_PHASE_SHIFTING')
    parser.add_argument('calib_type', type=str, nargs='?',help='Calibration type of camera: A(default)-automatic, M-manual')

    args = parser.parse_args()

    if args.action == None:
        args.action = input('What action will be performed? S-scanning, CC-camera calibration, PC-projector calibration: ')
    if args.pattern == None:
        args.pattern = int(input('Pattern for scanning:'
                            '0-WHITE, 1-BINARY, 2-STRIPE, 3-GRAY_CODE, 4-PHASE_SHIFTING, 5(default)-GRAY_CODE_AND_PHASE_SHIFTING: '))
        if args.pattern == None:
            args.pattern = 5
    if args.calib_type == None:
        args.calib_type = input('Calibration type of camera: A(default)-automatic, M-manual: ')
        if args.calib_type == None:
            args.pattern = 'A'
    main(args)

    # patterns = sl.Patterns() # шаблони
    # patternsArr = patterns.genetare(patternCode,dsize) # шаблоните
    # for key, pattr in patternsArr.items():
    #     for i,img in enumerate(pattr):
    #         cv.imshow(key,img)
    #         cv.waitKey(0)
    #     cv.destroyAllWindows()
