"""
Structure light 3D scanner

@TODO:
    * Да се създаде нов клас processing3D, който да съдържа методите за записване и зареждане на 3D данните. Може би ще съдържа и генерирането на Mesh
    * функцията в reconstruct3D за четене да изображенията да се преместви в cameraPi
    * Reconstruct3D.filterGrayCode - не работи добре, да се оправи
    * За получаване на минимална разлика между пиксели трябва да се направи Lagrange Interpolation
        * https://aikiddie.wordpress.com/2017/05/24/depth-sensing-stereo-image/
    * Web app
        * да се направят фукции връщане point cloud, mesh - файловеи формати .ply, .stl, .obj. Кухи, ако не се направи имплементация
        * връща изображенията в архив и после погат да се пуснат в MeshLap.
    * Тестът за точност може да бъде между резултата в MeshLap и сканиране на ДиТра
"""
import cv2 as cv
import numpy as np
import structuredlight as sl
import math
import argparse
import time

def scan(args): # Сканиране
    start = time.time()
    scanY.scan(args.pattern, args.stepSize)
    print('exec time: ', time.time()-start)

def stereoCalib(args): # Стерео калибриране
    scanY.stereoCalibrate(args.chessboardSize)

def cCalib(args): # Калибриране на камерата
    scanY.cameraCalibrate((args.pHeight,args.pWidth),args.chessboardSize, args.chessboardSize, args.calib_type, args.calibImgCnt)

def pCalib(args): # Калибриране на проектора
    scanY.projectorCalibrate((args.pHeight,args.pWidth),args.chessboardSize, args.calibImgCnt)

def main():
    print("main")

    # patternsArr = scanY.patterns.genetare(3,(360,615),(6,8)) # шаблоните
    # for key, pattr in patternsArr.items():
    #     for i,img in enumerate(pattr):
    #         cv.imwrite("./Projector_Calib" + "/tmpPattern.jpg", img)
    #         # img = scanY.cameraPi.undistortImage(img,scanY.cameraPi.stereoCalibrationRes["cameraMatrix"],
    #         #                           scanY.cameraPi.stereoCalibrationRes["cameraDistortion"], None,
    #         #                           scanY.cameraPi.stereoCalibrationRes["cRoi"])
    #         print(img.shape)
    #         cv.imshow(key,img)
    #         cv.waitKey(0)
    #     cv.destroyAllWindows()

if __name__=="__main__":
    scanY = sl.StructuredLight()

    parser = argparse.ArgumentParser(
        description='Scan-y 3D structured light scanner\n',
        formatter_class=argparse.RawTextHelpFormatter)
    subparsers = parser.add_subparsers()

    #scan
    parser_scan = subparsers.add_parser(
        'scan', help='Scan object')
    parser_scan.add_argument('pattern', type=int, nargs='?',default = 3, help='Pattern for scanning.'
                                '0-WHITE, 1-BLACK, 2-OPENCV_GRAY_CODE, 3(default)-MANUAL_GRAY_CODE, 4-PHASE_SHIFTING, 5-GRAY_CODE_AND_PHASE_SHIFTING, 6-BINARY, 7-STRIPE,')
    parser_scan.add_argument('stepSize', type=int, nargs='?', choices=range(1,200), metavar="stepSize[1-200]",default = 10,
                                help='Size of step of step motor.(1 step=1.8 degrees). Range between 1 and 200. Default:10')
    parser_scan.set_defaults(func=scan)

    #Stereo calibration
    parser_stereoCalib = subparsers.add_parser(
        'stereoCalib', help='Stereo calibration')
    parser_stereoCalib.add_argument('chessboardSize', type=int, nargs='?',default = (6,8), help='Chessboard size. Default: (6,8)')
    parser_stereoCalib.set_defaults(func=stereoCalib)

    #Camera calibration
    parser_cCalib = subparsers.add_parser(
        'cCalib', help='Camera calibration')
    parser_cCalib.add_argument('calib_type', type=str, nargs='?',choices=list(['A','M']),default = 'A',help='Calibration type of camera: A(default)-automatic, M-manual')
    parser_cCalib.add_argument('pHeight', type=int, nargs='?',default = 360, help='Projector height. Default: 360')
    parser_cCalib.add_argument('pWidth', type=int, nargs='?',default = 615, help='Projector width. Default: 615')
    parser_cCalib.add_argument('chessboardSize', type=int, nargs='?',default = (6,8), help='Chessboard size. Default: (6,8)')
    parser_cCalib.add_argument('chessBlockSize', type=int, nargs='?',default = 16, help='Chessboard block size. Default: 16mm')
    parser_cCalib.add_argument('calibImgCnt', type=int, nargs='?',default = 20, help='Count of calibration images. Default: 20')
    parser_cCalib.set_defaults(func=cCalib)

    #Projector calibration
    parser_pCalib = subparsers.add_parser(
        'pCalib', help='Projector calibration')
    parser_pCalib.add_argument('pHeight', type=int, nargs='?',default = 360, help='Projector height. Default: 360')
    parser_pCalib.add_argument('pWidth', type=int, nargs='?',default = 615, help='Projector width. Default: 615')
    parser_pCalib.add_argument('chessboardSize', type=int, nargs='?',default = (6,8), help='Chessboard size. Default: (6,8)')
    parser_pCalib.add_argument('calibImgCnt', type=int, nargs='?',default = 20, help='Count of calibratin images. Default: 20')
    parser_pCalib.set_defaults(func=pCalib)

    args = parser.parse_args()
    if hasattr(args, 'func'):
        args.func(args)
    else:
        parser.print_help()
    main()
