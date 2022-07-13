from picamera import PiCamera
import numpy as np
from natsort import natsorted
import cv2 as cv
import glob
import os
from .projector import Projector
from scipy import ndimage

"""
    Генериране на шаблони за сканиране
"""
class CameraPi:

    CALIBRATION_DIR = "./Camera_Calib" # Директория съдържаща снимките за калибриране и файла с резултата
    CALIBRATION_FILE = "/CalibResult.json" # Файл с резултата от калибрирането
    CALIBRATION_RES_IMAGE = "/CalibResult.jpg" # Файл с резултата от калибрирането

    STEREO_CALIBRATION_DIR = "./Stereo_Calib" # Резултатът от стерео калибрирането се записва в главната директория
    STEREO_CALIBRATION_FILE = "/StereoCalibResult.json" # Файл с резултата от калибрирането

    # Инициализиране на необходимите параметри на камерата
    def __init__(self):
        # зареждане на вътрешните параметри на камерата
        self.cCalibrationRes = self.readCalibrationResult(self.CALIBRATION_DIR)
        # зареждане на вътрешните параметри от стерео калибрирането
        self.stereoCalibrationRes = self.readStereoCalibrationResult()

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
        Връщане на резултат от калибрирането според подадената директория, в която ще се използва
    """
    def getUndistortCalibrationRes(self, dir):
        calibRes = None
        # Ако е калибриране на прожектора, то се прилага калибрирането на камерата
        if dir.startswith(Projector.CALIBRATION_DIR):
            calibRes= {
                "matrix" : self.cCalibrationRes["matrix"],
                "distortion" : self.cCalibrationRes["distortion"],
                "newCameraMatrix": self.cCalibrationRes["newCameraMatrix"],
                "roi": self.cCalibrationRes["roi"]
                }
        # Ако не е калибриране на камерата или стерео, то се опитва да се приложи резултата от стерео калибрирането
        elif (not dir.startswith(self.CALIBRATION_DIR) and
                not dir.startswith(self.STEREO_CALIBRATION_DIR)):
            # При стерео калибрацията няма newCameraMatrix и се подава None
            # тогава се оправяне на изкривяването се използва само matrix
            calibRes= {
                "matrix" : self.stereoCalibrationRes["cameraMatrix"],
                "distortion" : self.stereoCalibrationRes["cameraDistortion"],
                "newCameraMatrix": None,
                "roi": self.stereoCalibrationRes["cRoi"]
                }
        return calibRes

    # Зарежда и "изправя" изображението, ако е приложимо
    def loadImage(self, fname,flag=cv.IMREAD_COLOR):
        img = cv.imread(fname, flag)
        calibRes = self.getUndistortCalibrationRes(fname)

        # if calibRes != None:
        #     img = self.undistortImage(img,calibRes)
        return img

    #Прочита изображенията за определен шаблон
    def loadPatternImages(self, dir, patternCode,  scan_no, readType = cv.IMREAD_GRAYSCALE, img_no='?'):
        # Зареждат се всички изображения, които отговорят на шаблона
        # img_no - ?(точно един символ),*(0 или повече символи),конретно чисто(зарежда изображението с конкретен номер)
        imgsNames = natsorted(glob.glob('{0}/image{1}{2}{3}.jpg'.format(dir,scan_no,patternCode,img_no)))
        imgs = [] # масив със заредените изображения
        img = None
        for fname in imgsNames:
            img = self.loadImage(fname,readType)
            imgs.append(img)
        return np.array(imgs)

    """
        Стерео калибриране на камерата и проектора.
        patt - шаблонът на шахматна дъска, който е бил прожектиран и заснет от камерата. Ще бъде представен
               "като това, което вижда проектора, ако беше камера".
        projCalibResults - зарежда се в structuredlight, защото трябва да се вземе размера на проектора за генериране на шаблон
    """
    def stereoCalibrate(self, chessboardSize, projCalibResults):
        camCalibResults = self.cCalibrationRes
        objpoints, camImgpoints, projImgpoints = self.stereoFindChessboardCorners(chessboardSize)# Точките на дъската, в изображението на камерата и в проектора.

        # cv.CALIB_USE_INTRINSIC_GUESS - преизчислява cameraMatrix, cameraDistortion, projectorMatrix, projectorDistortion
        # r_matrix, t_vecs - rotation и transition между камерата и проектораl. Това основно се използва за
        # essentialMatrix, fundamentalMatrix - 3x3 матрици. Свързват съответните точки в стеро изображение. essentialMatrix може да се разглежда като предшественик на fundamentalMatrix
        ret, cameraMatrix, cameraDistortion, projectorMatrix, projectorDistortion, r_matrix, t_vecs,  essentialMatrix, fundamentalMatrix = \
            cv.stereoCalibrate(objpoints, camImgpoints, projImgpoints, camCalibResults["matrix"], camCalibResults["distortion"],
                        projCalibResults["matrix"], projCalibResults["distortion"], tuple(camCalibResults["shape"]),flags = cv.CALIB_USE_INTRINSIC_GUESS)

        # camRectTransMat, projRectTransMat, camProjectionMatrix, projProjectionMatrix -
        # disparityToDepthMatrix - 4x4 матрица трансформираща преспективата(Q) за преобразуване на пиксели disparity в съответните [x, y, z]
        # cRoi, pRoi - използва се изрязване само да валидната област на изображението след калибриране, но не работи вярно и дава координати (0,0),(0,0), т.е. нищо не е валидно.
        camRectTransMat, projRectTransMat, camProjectionMatrix, projProjectionMatrix, disparityToDepthMatrix, cRoi, pRoi = \
            cv.stereoRectify(cameraMatrix, cameraDistortion, projectorMatrix, projectorDistortion, tuple(camCalibResults["shape"]), r_matrix, t_vecs)

        # запиване на резултата от калибрирането
        stereoCalibrationRes = {
            "cShape" : camCalibResults["shape"],
            "pShape" : projCalibResults["shape"],
            "cameraMatrix": cameraMatrix,
            "cameraDistortion": cameraDistortion,
            "cRoi": cRoi,
            "projectorMatrix": projectorMatrix,
            "projectorDistortion": projectorDistortion,
            "pRoi": pRoi,
            "r_matrix": r_matrix,
            "t_vecs": t_vecs,
            "disparityToDepthMatrix": disparityToDepthMatrix
            }
        self.writeStereoCalibrationResult(stereoCalibrationRes)

    """
        Калибриране на камерата.
        Източник: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        chessboardSize: Размерите на шаха.Трябва да са точни иначе findChessboardCorners ще върне false.
        (8,6)
        chessBlockSize: Ширина на едно кврадче от дъската
        size: Размера на изображението. Попълва се при проектора.
        !!!ВАЖНО: В снимките за калибиране трябва да са такива, при които дъската е много близо до ръба,
                  защото иначе калибриране е грешно и изкривява крайното изображениета
            * https://answers.opencv.org/question/28438/undistortion-at-far-edges-of-image/
    """
    def calibrate(self, calibrationDir, chessboardSize, chessBlockSize, size=None):
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
        images = natsorted(glob.glob('{0}/{1}'.format(calibrationDir,'image*.jpg')))
        for fname in images:
            print(fname)
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
                # @TODO: Пременно е махнато триенето на изображенията без шаблон
                pass #os.remove(fname)

        if lastImageWithPattern is not None and matched_pattern_cnt >= 12:
            img = self.loadImage(lastImageWithPattern)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Калибриране на камерата(ъгли в обект,ъгли в изображението,размерите на изображението в обратен
            # ред(от -1)(?),без матрица на изображението,без коеф. на изкривяване)
            # return: Флаг за успех(ret), матрица на камерата(matrix), изкривяване(distortion),
            # изходен вектор на ротиране(r_matrix),изходен вектор на транслиране(t_vecs)
            ret, matrix, distortion, r_matrix, t_vecs = cv.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None)

            h,w = img.shape[:2] # размерите на изображението
            newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(matrix,
                                                                distortion,
                                                                (w,h),0,(w,h))

            # Изчисляване на грешката
            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv.projectPoints(objpoints[i], r_matrix[i], t_vecs[i], matrix, distortion)
                error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
                mean_error += error
            err = mean_error/len(objpoints)
            print( "total error: {}".format(err))
            # запиване на резултата от калибрирането
            calibrationRes = {
                "shape" : gray.shape if size is None else size, # записва се подадения ръчно размер, иначе се взима този на изображението(използва се за проектора)
                "matrix": matrix,
                "distortion": distortion,
                "newCameraMatrix": newCameraMatrix,
                "roi": roi,
                "error": err
                }
            self.writeCalibrationResult(calibrationDir,calibrationRes)

            # Прочитане на резултата от калибрирането
            calibrationRes = self.readCalibrationResult(calibrationDir)
            img = self.undistortImage(img, calibrationRes, calibrationDir)
        else:
            raise ValueError('Not enough matched patterns({0})!'.format(matched_pattern_cnt))

    # Намиране на ъглите на квадратите
    # return -> ret: дали са намерени ъгли, corners: координати на ъглите
    def findChessboardCorners(self, fname, chessboardSize):
        img = self.loadImage(fname, cv.IMREAD_GRAYSCALE) #чете изображението и преображуване в черно-бяло
        # Намиране на ъглите на квадратите(сиво изображение, размер на дъска, без флагове)
        # ret: дали са намерени ъгли, corners: координати на ъглите
        return cv.findChessboardCorners(img,(chessboardSize[0],chessboardSize[1]), None, 0)

    # Стерео намиране на ъглите на квадратите
    def stereoFindChessboardCorners(self, chessboardSize):
        # !!!ВАЖНО: горе с calibrate, има описания какво означават тези редове
        objp = np.zeros((chessboardSize[0]*chessboardSize[1],3), np.float32)
        objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # Взимане на imgpoints за камерата
        fname = natsorted(glob.glob('{0}/{1}'.format(self.STEREO_CALIBRATION_DIR,'image*.jpg')))[0]
        img = self.loadImage(fname)
        ret, corners = cv.findChessboardCorners(img,(chessboardSize[0],chessboardSize[1]), None)
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)
        else:
            raise ValueError('Not good image for stereo calibration!')
        camImgpoints = imgpoints

        # Взимане на imgpoints за проектора. Тук шаблона се представя като това, което е заснето от камерата
        img = self.loadImage(Projector.CALIBRATION_DIR + Projector.TMP_PATTERN_FILE_NAME)
        img = ndimage.rotate(img, 90) # Завъртане, защото проектора го използва вертикално, за да го покаже хоризонтално
        ret, corners = cv.findChessboardCorners(img,(chessboardSize[0],chessboardSize[1]), None)
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)
        else:
            raise ValueError('Not good image for stereo calibration!')
        projImgpoints = imgpoints

        return (objpoints, camImgpoints, projImgpoints)

    # Калибриране на изображението
    def undistortImage(self, img, calibrationRes, calibrationDir=None):
        # Премахване на изкривяването
        dst = cv.undistort(img, calibrationRes["matrix"], calibrationRes["distortion"], img.shape[:1], calibrationRes["matrix"])#newCameraMatrix

        # СПИРА СЕ ИЗРЯЗВАНЕТО ПО ROI, ЗАЩОТО ВИНАГИ ТРЯБВА ПЪЛНОТО ИЗОБРАЖЕНИЕ
        # СЪЩО stereoRectify ВРЪЩА ВИНАГИ 0,0,0,0 И ТРЯБВА ДА СЕ ОПРАВИ, АКО ЩЕ СЕ ПРИЛАГА ИЗРЯЗВАНЕ НА НЕВАЛИДНИ ПИКСЕЛИ
        # Изрязване на изображението. Всяка стойност е масив, затова с flatten се преобразува от 2D в 1D масив
        # x,y,w,h = calibrationRes["roi"].flatten().astype(int)
        # dst = dst[y:y+h, x:x+w]

        if calibrationDir != None:
            cv.imwrite(calibrationDir + self.CALIBRATION_RES_IMAGE, dst)
            print("Done!")
        return dst

    """
        Записесне на резултатите от калбирането във файл
    """
    def writeCalibrationResult(self, calibrationDir, calibrationRes):
        # отваряне на файл за записване на резултата от калибрацията
        fs = cv.FileStorage(calibrationDir + self.CALIBRATION_FILE, cv.FILE_STORAGE_WRITE)
        # Параметри на камерата
        fs.write('shape', calibrationRes["shape"])
        fs.write('matrix', calibrationRes["matrix"])
        fs.write('distortion', calibrationRes["distortion"])
        fs.write('newCameraMatrix', calibrationRes["newCameraMatrix"])
        fs.write('roi', calibrationRes["roi"])
        fs.write('error', calibrationRes["error"])
        fs.release()

    """
        Записесне на резултатите от stereo калбирането във файл
    """
    def writeStereoCalibrationResult(self, stereoCalibrationRes):
        # отваряне на файл за записване на резултата от калибрацията
        fs = cv.FileStorage(self.STEREO_CALIBRATION_DIR + self.STEREO_CALIBRATION_FILE, cv.FILE_STORAGE_WRITE)
        # Параметри на камерата
        fs.write('cShape', stereoCalibrationRes["cShape"])
        fs.write('pShape', stereoCalibrationRes["pShape"])
        fs.write('cameraMatrix', stereoCalibrationRes["cameraMatrix"])
        fs.write('cameraDistortion', stereoCalibrationRes["cameraDistortion"])
        fs.write('cRoi', stereoCalibrationRes["cRoi"])
        fs.write('projectorMatrix', stereoCalibrationRes["projectorMatrix"])
        fs.write('projectorDistortion', stereoCalibrationRes["projectorDistortion"])
        fs.write('pRoi', stereoCalibrationRes["pRoi"])
        fs.write('r_matrix', stereoCalibrationRes["r_matrix"])
        fs.write('t_vecs', stereoCalibrationRes["t_vecs"])
        fs.write('disparityToDepthMatrix', stereoCalibrationRes["disparityToDepthMatrix"])
        fs.release()

    """
        Прочитане на резултатите от калбирането
    """
    def readCalibrationResult(self, calibrationDir):
        calibResulPath = calibrationDir + self.CALIBRATION_FILE
        file_exists = os.path.exists(calibResulPath)

        calibrationRes = None
        if file_exists:
            # отваряне на файл за записване на резултата от калибрацията
            fs = cv.FileStorage(calibResulPath, cv.FILE_STORAGE_READ)
            # Параметри на камерата
            calibrationRes = {
                "shape" : tuple(fs.getNode('shape').mat().astype(int).flatten()),
                "matrix" : fs.getNode('matrix').mat(),
                "distortion" : fs.getNode('distortion').mat(),
                "newCameraMatrix" : fs.getNode('newCameraMatrix').mat(),
                "roi" : fs.getNode('roi').mat(),
                "error" : fs.getNode('error').real()
                }
        return calibrationRes

    """
        Прочитане на резултатите от stereo калбирането
    """
    def readStereoCalibrationResult(self):
        stereoCalibResulPath = self.STEREO_CALIBRATION_DIR + self.STEREO_CALIBRATION_FILE
        file_exists = os.path.exists(stereoCalibResulPath)

        stereoCalibrationRes = None
        if file_exists:
            # отваряне на файл за записване на резултата от калибрацията
            fs = cv.FileStorage(stereoCalibResulPath, cv.FILE_STORAGE_READ)
            # Параметри на камерата
            stereoCalibrationRes = {
                "cShape" : tuple(fs.getNode('cShape').mat().astype(int).flatten()),
                "pShape" : tuple(fs.getNode('pShape').mat().astype(int).flatten()),
                "cameraMatrix" : fs.getNode('cameraMatrix').mat(),
                "cameraDistortion" : fs.getNode('cameraDistortion').mat(),
                "cRoi" : fs.getNode('cRoi').mat(),
                "projectorMatrix" : fs.getNode('projectorMatrix').mat(),
                "projectorDistortion" : fs.getNode('projectorDistortion').mat(),
                "pRoi" : fs.getNode('pRoi').mat(),
                "r_matrix" : fs.getNode('r_matrix').mat(),
                "t_vecs" : fs.getNode('t_vecs').mat(),
                "disparityToDepthMatrix" : fs.getNode('disparityToDepthMatrix').mat()
                }
        return stereoCalibrationRes
