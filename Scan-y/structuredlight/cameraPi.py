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
    CALIBRATION_FILE = "/CalibResult.json" # Файл с резултата от калибрирането
    CALIBRATION_RES_IMAGE = "/CalibResult.jpg" # Файл с резултата от калибрирането

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
        !!!ВАЖНО: В снимките за калибиране трябва да такива, при които дъската е много близо до ръба, защото иначе калибриране е грешно и изкривява крайното изображениета
            * https://answers.opencv.org/question/28438/undistortion-at-far-edges-of-image/
    """
    def calibrate(self, calibrationDir, chessboardSize, chessBlockSize):
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
        
        matched_pattern_cnt = 0 #брой намерени шаблони. Трябва да са поне 12, за да е коректно калибрирането.

        # взима имената на изображениета *.jpg от директорията сортирани по естествен начин
        images = natsorted(glob.glob("./"+calibrationDir + '/image*.jpg'))
        for fname in images:
            # Намиране на ъглите на квадратите
            ret, corners = self.findChessboardCorners(fname, chessboardSize)
            if ret == True:
                matched_pattern_cnt += 1
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

        if lastImageWithPattern is not None and matched_pattern_cnt >= 12:
            lastImageWithPattern='./Camera_Calib/image0White0.jpg'
            img = cv.imread(lastImageWithPattern)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Калибриране на камерата(ъгли в обект,ъгли в изображението,размерите на изображението в обратен
            # ред(от -1)(?),без матрица на изображението,без коеф. на изкривяване)
            # return: Флаг за успех(ret), матрица на камерата(matrix), изкривяване(distortion),
            # изходен вектор на ротиране(r_vecs),изходен вектор на транслиране(t_vecs)
            ret, matrix, distortion, r_vecs, t_vecs = cv.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None)
            
            # Изчисляване на грешката
            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv.projectPoints(objpoints[i], r_vecs[i], t_vecs[i], matrix, distortion)
                error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
                mean_error += error
            err = mean_error/len(objpoints)
            print( "total error: {}".format(err))
            # запиване на резултата от калибрирането
            calibrationRes = {
                "shape" : gray.shape,
                "matrix": matrix,
                "distortion": distortion,
                "r_vecs": r_vecs,
                "t_vecs": t_vecs,
                "error": err
                }
            self.writeCalibrationResult(calibrationDir,calibrationRes)

            # Прочитане на резултата от калибрирането
            calibrationRes = self.readCalibrationResult(calibrationDir)
            img = self.undistortImage(img, calibrationDir, calibrationRes)
            
        else:
            raise ValueError('Not enough matched patterns({0})!'.format(matched_pattern_cnt))

    # Намиране на ъглите на квадратите(сиво изображение, размер на дъска, без флагове)
    # return -> ret: дали са намерени ъгли, corners: координати на ъглите
    def findChessboardCorners(self, fname, chessboardSize):
        img = cv.imread(fname, cv.IMREAD_GRAYSCALE) #чете изображението и преображуване в черно-бяло
        # Намиране на ъглите на квадратите(сиво изображение, размер на дъска, без флагове)
        # ret: дали са намерени ъгли, corners: координати на ъглите
        return cv.findChessboardCorners(img,(chessboardSize[0],chessboardSize[1]), None)

    # Калибриране на изображението
    def undistortImage(self, img, calibrationDir, calibrationRes):
        h,w = img.shape[:2] # размерите на изображението
        newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(calibrationRes["matrix"],
                                                            calibrationRes["distortion"],
                                                            (w,h),0,(w,h))
        # Премахване на изкривяването
        dst = cv.undistort(img, calibrationRes["matrix"],
                           calibrationRes["distortion"], None, newCameraMatrix)
        # Изрязване на изображението
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imwrite("./"+ calibrationDir + self.CALIBRATION_RES_IMAGE, dst)
        print("Done!")
        return dst

    """
        Записесне на резултатите от калбирането във файл
    """
    def writeCalibrationResult(self, calibrationDir, calibrationRes):
        # отваряне на файл за записване на резултата от калибрацията
        fs = cv.FileStorage("./"+ calibrationDir + self.CALIBRATION_FILE, cv.FILE_STORAGE_WRITE)
        # Параметри на камерата
        fs.write('shape', calibrationRes["shape"])
        fs.write('matrix', calibrationRes["matrix"])
        fs.write('distortion', calibrationRes["distortion"])
        fs.write('error', calibrationRes["error"])
        # fs.write('cam_r_vecs', camCalibration["r_vecs"])
        # fs.write('cam_t_vecs', camCalibration["r_vecs"])
        fs.release()

    """
        Прочитане на резултатите от калбирането
    """#np.asarray(cameraCalib["matrix"]) #възтановявне на типа да бъде Numpy array
    def readCalibrationResult(self, calibrationDir):
        # отваряне на файл за записване на резултата от калибрацията
        fs = cv.FileStorage("./"+ calibrationDir + self.CALIBRATION_FILE, cv.FILE_STORAGE_READ)
        # Параметри на камерата
        calibrationRes = {
            "shape" : fs.getNode('shape').mat(),
            "matrix" : fs.getNode('matrix').mat(),
            "distortion" : fs.getNode('distortion').mat(),
            "error" : fs.getNode('error')
            # "r_vecs" : fs.getNode('cam_r_vecs').mat(),
            # "t_vecs" : fs.getNode('cam_t_vecs').mat()
            }
        return calibrationRes
