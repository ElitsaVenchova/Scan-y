from picamera import PiCamera
from time import sleep
import numpy as np
from natsort import natsorted
import cv2 as cv
import glob
#write to file
import json
from json import JSONEncoder
import ast

"""
    Генериране на шаблони за сканиране
"""
class CameraPi:

    CALIBRATION_DIR = "Calib" # Директория съдържаща снимките за калибриране

    # Задаване на OUT pin-овете и размер на стъпката
    def __init__(self, chessboardSize):
        # Размер на шахматната дъска. Трябва да са точен иначе findChessboardCorners ще върне false.
        self.chessboardSize = chessboardSize

    # Клас наследяващат JSONEncoder, за да може да се сериализира Numpy array
    class NumpyArrayEncoder(JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            return JSONEncoder.default(self, obj)

    """
        Прави снимка с камерата и връща името на jpg файла.
    """
    def takePhoto(self, dir, imageInd):
        imageFullName = '{0}/image{1}.jpg'.format(dir,imageInd)
        with PiCamera() as camera:
        # preview е само за debug
            camera.stop_preview()
            camera.capture(imageFullName)
            camera.stop_preview()
        return imageFullName

    """
        Калибриране на камерата.
        Източник: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        dsize: Размерите на шаха.Трябва да са точни иначе findChessboardCorners ще върне false.
        (8,6)
    """
    def calibrate(self):
        # Критерии за спиране на търсенето. Използва се в cornerSubPix, което намира по-точно ъглите на дъската
        # (type:COUNT,EPS or COUNT + EPS(2+1),maxCount iteration,
        # epsilon: при каква точност или промяна на стойност алгоритъма спира)
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # масив нули с редове #ъгли и 3 колони
        objp = np.zeros((self.chessboardSize[0]*self.chessboardSize[1],3), np.float32) #[56][3]
        # прави масива като (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        # един вид все едно в реалния свят са 1,2,3,...
        objp[:,:2] = np.mgrid[0:self.chessboardSize[0],0:self.chessboardSize[1]].T.reshape(-1,2)

        # Масиви за съхранение точките на обекта и точките в избражението
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # взима имената на изображениета *.jpg от директорията
        lastImageWithPattern = None #после изображение с намерен шаблон //После за тест на калибрирането
        images = natsorted(glob.glob("./"+self.CALIBRATION_DIR + '/image*.jpg'))
        for fname in images:
            print(fname)
            img = cv.imread(fname) #чете изображението
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) #преображуване в черно-бяло

            # Намиране на ъглите на квадратите(сиво изображение, размер на дъска, без флагове)
            # ret: дали са намерени ъгли, corners: координати на ъглите
            ret, corners = cv.findChessboardCorners(gray,(self.chessboardSize[0],self.chessboardSize[1]), None)
            if ret == True:
                lastImageWithPattern = fname
                print("Found pattern in " + fname)
                # намира по-точните пиксел на ъгълите(изобр.,ъгли, 1/2 от страничната
                # дължина за търсене(???), няма zeroZone,критерии за спиране)
                corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)

                # Добавят се координатите на ъглите. После за калибрирането
                objpoints.append(objp)
                imgpoints.append(corners)

                cv.drawChessboardCorners(img, (self.chessboardSize[0],self.chessboardSize[1]), corners2, ret)
                # cv.imwrite('Corners' + fname, img) #записва изображението с намерените ъгли на дъската

        if lastImageWithPattern is not None:
            # Калибриране на камерата(ъгли в обект,ъгли в изображението,размерите на изображението в обратен
            # ред(от -1)(?),без матрица на изображението,без коеф. на изкривяване)
            # return: Флаг за успех(ret), матрица на камерата(matrix), изкривяване(distortion),
            # изходен вектор на ротиране(r_vecs),изходен вектор на транслиране(t_vecs)
            ret, matrix, distortion, r_vecs, t_vecs = cv.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None)

            # запиване на резултата от калибрирането
            resCalibration = {
                "ret" : ret,
                "matrix": matrix,
                "distortion": distortion,
                "r_vecs": r_vecs,
                "t_vecs": t_vecs
                }
            json.dump(resCalibration, open("./"+self.CALIBRATION_DIR + "/CameraCalibrationResult.txt",'w'), cls=self.NumpyArrayEncoder)

            # Прочитане на резултата от калибрирането
            cameraCalib = eval(open("./"+self.CALIBRATION_DIR + "/CameraCalibrationResult.txt",'r').read())
            cameraCalib["matrix"] = np.asarray(cameraCalib["matrix"]) #възтановявне на типа да бъде Numpy array
            cameraCalib["distortion"] = np.asarray(cameraCalib["distortion"]) #възтановявне на типа да бъде Numpy array

            img = cv.imread(lastImageWithPattern)
            h,w = img.shape[:2] # размерите на изображението
            newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(matrix,
                                                                distortion,
                                                                (w,h),0,(w,h))

            # Премахване на изкривяването
            dst = cv.undistort(img, matrix,
                               distortion, None, newCameraMatrix)

            # Изрязване на изображението
            x,y,w,h = roi
            dst = dst[y:y+h, x:x+w]
            cv.imwrite("./"+self.CALIBRATION_DIR + '/calibResult.jpg', dst)
            print("Done!")
