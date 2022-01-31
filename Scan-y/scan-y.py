"""
Structure light 3D scanner

@TODO:
    * Да се направи subparsers за Калибриране камера, Калибриране проектор и Сканиране и да се попълват args само за тях
        * Пример: https://github.com/kamino410/phase-shifting
    * Нова логика за scan:
        * Шаблони:
            * Gray code да се направи чрез cv2.structured_light_GrayCodePattern.create(gc_width, gc_height)
            * Phase phift се запазва
            * Добавяне на бял и черен шаблон
        * Декодиране:
            * (~)С graycode.getProjPixel(gc_imgs, x, y) да се вземе връзката между шаблона и заснетото изображение
            * (#)С позната формула за argtan да се загърне фазата
        * Резултат:
            * (#) + (~)*2pi            
    * Останали задачи
        * Да се измисли как да използвам калибрирането.
            * Формилата е (x,y)image = K(R*(x,y)real + T), K-матрица на калибриране, R-матрица на ротация, T-транслиращ вектор.
            * Да се види реализацията на http://mesh.brown.edu/byo3d/source.html
            * Workflow - Генерират се шаблоните->Undistortion с OpenCV на шаблоните->Прожектират се->Заснемат се изображенията->Undistortion с OpenCV на изображенията->Резултатът се пуска за обработка
        * Да се направи сканирането!
            * Има алгоритъм/ми за напрасване на една гледна точка към друга(за получаване на панорама)
            * са се прочете как се прави point cloud - .xyz(?)
            * да се прочете как от point cloud се прави point mesh - файловеи формати .ply, .stl, .obj
    * Web app
"""
import cv2 as cv
import numpy as np
import structuredlight as sl
import math
import argparse

def main(args):
    pSize =(615,360) #размер на проектора
    chessboardSize = (8,6)
    chessBlockSize = 16 # mm

#     patterns = sl.Patterns() # шаблони
#     patternsArr = patterns.genetare(6,pSize,chessboardSize) # шаблоните
#     for key, pattr in patternsArr.items():
#         for i,img in enumerate(pattr):
#             print(img.shape)
#             cv.imshow(key,img)
#             cv.waitKey(0)
#         cv.destroyAllWindows()

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
        args.pattern = input('Pattern for scanning:'
                            '0-WHITE, 1-BINARY, 2-STRIPE, 3-GRAY_CODE, 4-PHASE_SHIFTING, 5(default)-GRAY_CODE_AND_PHASE_SHIFTING: ')
        print(args.pattern)
        if args.pattern == str():
            args.pattern = 5
        else:
            args.pattern = int(args.pattern)
    if args.calib_type == None:
        args.calib_type = input('Calibration type of camera: A(default)-automatic, M-manual: ')
        if args.calib_type == None:
            args.pattern = 'A'
    main(args)
