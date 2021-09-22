# take photos imports
from picamera import PiCamera
from time import sleep
# calibration imports
import numpy as np
import cv2 as cv
import glob
#write to file
import json
import ast

# take 10 photos and save in current dir
##with PiCamera() as camera:
##    camera.start_preview(alpha=230,fullscreen=False,window=(50,80,1000,1200))
##    for i in range(10):
##        sleep(10)
##        camera.capture('image%s.jpg' % i)
##    camera.stop_preview()

# Размерите на шаха.
# Трябва да са точни иначе findChessboardCorners ще върне false.
CHECKERBOARD = (8,6)

# Критерии за спиране на търсенето. Използва се в cornerSubPix,
# което намира по-точно ъглите на дъската
# (type:COUNT,EPS or COUNT + EPS(2+1),maxCount iteration,
# epsilon: при каква точност или промяна на стойност алгоритъма спира)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# масив нули с редове #ъгли и 3 колони
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32) #[56][3]
# прави масива като (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# един вид все едно в реалния свят са 1,2,3,...
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)

# Масиви за съхранение точките на обекта и точките в избражението
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# взима имената на изображениета *.jpg от директорията
lastImageWithPattern = None #после изображение с намерен шаблон //После за тест на калибрирането
images = glob.glob('*.jpg')
for fname in images:
    img = cv.imread(fname) #чете изображението
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) #преображуване в черно-бяло
    
    # Намиране на ъглите на квадратите(сиво изображение, размер на дъска, без флагове)
    # ret: дали са намерени ъгли, corners: координати на ъглите
    ret, corners = cv.findChessboardCorners(gray,(CHECKERBOARD[0],CHECKERBOARD[1]), None)
    if ret == True:
        lastImageWithPattern = fname
        print("Found pattern in " + fname)
        # намира по-точните пиксел на ъгълите(изобр.,ъгли, 1/2 от страничната
        # дължина за търсене(???), няма zeroZone,критерии за спиране)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        
        # Добавят се координатите на ъглите. После за калибрирането
        objpoints.append(objp)
        imgpoints.append(corners)

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
        "matrix": matrix.tolist(), #Numpy array is not serializable
        "distortion": distortion.tolist(), #Numpy array is not serializable
        "r_vecs": r_vecs,
        "t_vecs": t_vecs
        }
    json.dump(resCalibration, open("CameraCalibrationResult.txt",'w'))

# Прочитане на резултата от калибрирането
cameraCalib = literal_eval(open("CameraCalibrationResult.txt",'r').read())
cameraCalib["matrix"] = np.array(cameraCalib["matrix"]) #възтановявне на типа да бъде Numpy array
cameraCalib["distortion"] = np.array(cameraCalib["distortion"]) #възтановявне на типа да бъде Numpy array

# Displaying required output
print(" Ret:")
print(cameraCalib["ret"])

print("\n Camera matrix:")
print(cameraCalib["matrix"])
 
print("\n Distortion coefficient:")
print(cameraCalib["distortion"])
 
print("\n Rotation Vectors:")
print(cameraCalib["r_vecs"])
 
print("\n Translation Vectors:")
print(cameraCalib["t_vecs"])
