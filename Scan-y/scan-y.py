"""
Structure light 3D scanner

@TODO:
    * Собствена реалциация на gray code mapping.
    * Да се види реализацията на http://mesh.brown.edu/byo3d/source.html
    * Има алгоритъм/ми за напрасване на една гледна точка към друга(за получаване на панорама)
    * Web app
        * да се направят фукции връщане point cloud, mesh - файловеи формати .ply, .stl, .obj. Кухи, ако не се направи имплементация
        * връща изображенията в архив и после погат да се пуснат в MeshLap.
    * Коментиране на кода
    * Тестът за точност може да бъде между резултата в MeshLap и сканиране на ДиТра
"""
import cv2 as cv
import numpy as np
import structuredlight as sl
import math
import argparse

def scan(args):
    scanY.scan(args.pattern)

def stereoCalib(args):
    scanY.stereoCalibrate(args.chessboardSize)

def cCalib(args):
    scanY.cameraCalibrate(args.chessboardSize, args.chessboardSize, args.calib_type)

def pCalib(args):
    scanY.projectorCalibrate(args.chessboardSize)

def main():
    print("main")

    # patterns = sl.Patterns() # шаблони
    # patternsArr = patterns.genetare(6,pSize,chessboardSize) # шаблоните
    # for key, pattr in patternsArr.items():
    #     for i,img in enumerate(pattr):
    #         print(img.shape)
    #         cv.imshow(key,img)
    #         cv.waitKey(0)
    #     cv.destroyAllWindows()

if __name__=="__main__":
    chessBlockSize = 16 # mm

    parser = argparse.ArgumentParser(
        description='Scan-y 3D structured light scanner\n',
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('pHeight', type=int, nargs='?',default = 360, help='Projector height. Default: 360')
    parser.add_argument('pWidth', type=int, nargs='?',default = 615, help='Projector width. Default: 615')
    subparsers = parser.add_subparsers()

    #scan
    parser_scan = subparsers.add_parser(
        'scan', help='Scan object')
    parser_scan.add_argument('pattern', type=int, nargs='?',default = 2, help='Pattern for scanning.'
                                '0-WHITE, 1-BLACK, 2(default)-GRAY_CODE, 3-PHASE_SHIFTING, 4-GRAY_CODE_AND_PHASE_SHIFTING, 5-BINARY, 6-STRIPE,')
    parser_scan.set_defaults(func=scan)

    #Stereo calibration
    parser_stereoCalib = subparsers.add_parser(
        'stereoCalib', help='Stereo calibration')
    parser_stereoCalib.add_argument('chessboardSize', type=int, nargs='?',default = (6,8), help='Chessboard size. Default: (6,8)')
    parser_stereoCalib.set_defaults(func=stereoCalib)

    #Camera calibration
    parser_cCalib = subparsers.add_parser(
        'cCalib', help='Camera calibration')
    parser_cCalib.add_argument('calib_type', type=str, nargs='?',default = 'A',help='Calibration type of camera: A(default)-automatic, M-manual')
    parser_cCalib.add_argument('chessboardSize', type=int, nargs='?',default = (6,8), help='Chessboard size. Default: (6,8)')
    parser_cCalib.add_argument('chessBlockSize', type=int, nargs='?',default = 16, help='Chessboard block size. Default: 16mm')
    parser_cCalib.set_defaults(func=cCalib)

    #Projector calibration
    parser_pCalib = subparsers.add_parser(
        'pCalib', help='Projector calibration')
    parser_pCalib.add_argument('chessboardSize', type=int, nargs='?',default = (6,8), help='Chessboard size. Default: (6,8)')
    parser_pCalib.set_defaults(func=pCalib)

    args = parser.parse_args()
    if hasattr(args, 'func'):
        scanY = sl.StructuredLight((args.pHeight,args.pWidth))
        args.func(args)
    else:
        parser.print_help()
    main()
