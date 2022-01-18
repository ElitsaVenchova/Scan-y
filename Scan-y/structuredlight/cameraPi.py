from picamera import PiCamera
import numpy as np
from natsort import natsorted
import cv2 as cv
import glob
import os

"""
    Генериране на шаблони за сканиране
"""
class CameraPi:

    CALIBRATION_DIR = "Camera_Calib" # Директория съдържаща снимките за калибриране и файла с резултата
    CALIBRATION_FILE = "./"+ CALIBRATION_DIR + "/CameraCalibrationResult.json" # Файл с резултата от калибрирането

    """
        Прави снимка с камерата и връща името на jpg файла.
    """
    def takePhoto(self, dir, imageInd=""):
        imageFullName = '{0}/image{1}.jpg'.format(dir,imageInd)
        print(imageFullName)
        with PiCamera() as camera:
        # preview е само за debug
            camera.stop_preview()
            camera.capture(imageFullName)
            camera.stop_preview()
        return imageFullName

    """
        Калибриране на камерата.
        Източник: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        chessboardSize: Размерите на шаха.Трябва да са точни иначе findChessboardCorners ще върне false.
        (8,6)
        chessBlockSize: Ширина на едно кврадче от дъската
    """
    def calibrate(self, chessboardSize, chessBlockSize):
        # Критерии за спиране на търсенето. Използва се в cornerSubPix, което намира по-точно ъглите на дъската
        # (type:COUNT,EPS or COUNT + EPS(2+1),maxCount iteration,
        # epsilon: при каква точност или промяна на стойност алгоритъма спира)
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        lastImageWithPattern = None #после изображение с намерен шаблон //После за тест на калибрирането

        # масив нули с редове #ъгли и 3 колони
        objp = np.zeros((chessboardSize[0]*chessboardSize[1],3), np.float32) #[56][3]
        # прави масива като (size*0,size*0,size*0), (size*1,size*0,size*0), (size*2,size*0,size*0) ....,(size*6,size*5,size*0)
        # един вид все едно в реалния свят са size*1,size*2,size*3,...
        # size* е реалния размер на квадрат на дъската. Така лесно се преобразуват пискели в реални дълбини
        objp[:,:2] = chessBlockSize * np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

        # Масиви за съхранение точките на обекта и точките в избражението
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # взима имената на изображениета *.jpg от директорията сортирани по естествен начин
        images = natsorted(glob.glob("./"+self.CALIBRATION_DIR + '/image*.jpg'))
        for fname in images:
            # Намиране на ъглите на квадратите
            ret, corners = self.findChessboardCorners(fname, chessboardSize) 
            if ret == True:
                lastImageWithPattern = fname
                print("Found pattern in " + fname)
                # намира по-точните пиксел на ъгълите(изобр.,ъгли, 1/2 от страничната
                # дължина за търсене(???), няма zeroZone,критерии за спиране)
                # corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                # cv.drawChessboardCorners(img, (chessboardSize[0],chessboardSize[1]), corners2, ret)
                # cv.imwrite('Corners' + fname, img) #записва изображението с намерените ъгли на дъската

                # Добавят се координатите на ъглите. После за калибрирането
                objpoints.append(objp)
                imgpoints.append(corners)
            else:
                os.remove(fname)

        if lastImageWithPattern is not None:
            img = cv.imread(fname)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Калибриране на камерата(ъгли в обект,ъгли в изображението,размерите на изображението в обратен
            # ред(от -1)(?),без матрица на изображението,без коеф. на изкривяване)
            # return: Флаг за успех(ret), матрица на камерата(matrix), изкривяване(distortion),
            # изходен вектор на ротиране(r_vecs),изходен вектор на транслиране(t_vecs)
            ret, matrix, distortion, r_vecs, t_vecs = cv.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None)
            # запиване на резултата от калибрирането
            camCalibration = {
                "shape" : gray.shape,
                "matrix": matrix,
                "distortion": distortion,
                "r_vecs": r_ vecs,
                "t_vecs": t_vecs
                }
            projCalibration = {
                "shape" : np.array([0]),
                "matrix": np.array([0]),
                "distortion": np.array([0]),
                "r_vecs": np.array([0]),
                "t_vecs": np.array([0])
                }
            self.writeCalibrationResult(camCalibration, projCalibration)

            # Прочитане на резултата от калибрирането
            camCalibration, projCalibration = self.readCalibrationResult()
            img = self.calibrateImage(img, camCalibration)
            
    
    # Намиране на ъглите на квадратите(сиво изображение, размер на дъска, без флагове)
    # return -> ret: дали са намерени ъгли, corners: координати на ъглите
    def findChessboardCorners(self, fname, chessboardSize):
        img = cv.imread(fname, cv.IMREAD_GRAYSCALE) #чете изображението и преображуване в черно-бяло
        # Намиране на ъглите на квадратите(сиво изображение, размер на дъска, без флагове)
        # ret: дали са намерени ъгли, corners: координати на ъглите
        return cv.findChessboardCorners(img,(chessboardSize[0],chessboardSize[1]), None)

    # Калибриране на изображението
    def calibrateImage(self, img, camCalibration):
        h,w = img.shape[:2] # размерите на изображението
        newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(camCalibration["matrix"],
                                                            camCalibration["distortion"],
                                                            (w,h),0,(w,h))
        # Премахване на изкривяването
        dst = cv.undistort(img, camCalibration["matrix"],
                           camCalibration["distortion"], None, newCameraMatrix)
        # Изрязване на изображението
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imwrite("./"+self.CALIBRATION_DIR + '/calibResult.jpg', dst)
        print("Done!")
        return dst

    """
        Записесне на резултатите от калбирането във файл
    """
    def writeCalibrationResult(self, camCalibration, projCalibration):
        # отваряне на файл за записване на резултата от калибрацията
        fs = cv.FileStorage(self.CALIBRATION_FILE, cv.FILE_STORAGE_WRITE)
        # Параметри на камерата
        fs.write('cam_shape', camCalibration["shape"])
        fs.write('cam_matrix', camCalibration["matrix"])
        fs.write('cam_distortion', camCalibration["distortion"])
        # fs.write('cam_r_vecs', camCalibration["r_vecs"])
        # fs.write('cam_t_vecs', camCalibration["r_vecs"])
        # Параметри на проектора
        fs.write('proj_shape', projCalibration["shape"])
        fs.write('proj_matrix', projCalibration["matrix"])
        fs.write('proj_distortiont', projCalibration["distortion"])
        # fs.write('proj_r_vecs', projCalibration["r_vecs"])
        # fs.write('proj_t_vecs', projCalibration["t_vecs"])
        fs.release()


    """
        Прочитане на резултатите от калбирането
    """#np.asarray(cameraCalib["matrix"]) #възтановявне на типа да бъде Numpy array
    def readCalibrationResult(self):
        # отваряне на файл за записване на резултата от калибрацията
        fs = cv.FileStorage(self.CALIBRATION_FILE, cv.FILE_STORAGE_READ)
        # Параметри на камерата
        camCalibration = {
            "shape" : fs.getNode('cam_shape').mat(),
            "matrix" : fs.getNode('cam_matrix').mat(),
            "distortion" : fs.getNode('cam_distortion').mat()
            # "r_vecs" : fs.getNode('cam_r_vecs').mat(),
            # "t_vecs" : fs.getNode('cam_t_vecs').mat()
            }
        # Параметри на проектора
        projCalibration = {
            "shape" : fs.getNode('proj_shape').mat(),
            "matrix": fs.getNode('proj_matrix').mat(),
            "distortion": fs.getNode('proj_distortiont').mat()
            # "r_vecs": fs.getNode('proj_r_vecs').mat(),
            # "t_vecs": fs.getNode('proj_t_vecs').mat()
            }
        return (camCalibration, projCalibration)
