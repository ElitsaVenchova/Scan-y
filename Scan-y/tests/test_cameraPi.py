import pytest
import cv2 as cv
import glob
import os
import shutil
import sys
from natsort import natsorted
import glob
import numpy as np

from structuredlight.structuredlight import Patterns
from structuredlight.structuredlight import CameraPi
from structuredlight.structuredlight import Projector
from structuredlight.structuredlight import StructuredLight

TEST_DIR = './tests/test_files/camera'

"""
    Прочитане на резултатите от калбирането
"""
def readTestCalib(dir,fname):
    fs = cv.FileStorage(dir+fname, cv.FILE_STORAGE_READ)
    # Параметрите
    calibrationRes = {
        "shape" : fs.getNode('shape').mat().astype(int).flatten(),
        "matrix" : fs.getNode('matrix').mat(),
        "distortion" : fs.getNode('distortion').mat(),
        "newCameraMatrix" : fs.getNode('newCameraMatrix').mat(),
        "roi" : fs.getNode('roi').mat(),
        "error" : fs.getNode('error').real()
        }
    return calibrationRes

# Прочита резултатът от калибрирането само за фунцията за getUndistortCalibrationRes
def readTestCalibUndistor(calibrationRes):
    test_calib_file_res = {"matrix" : calibrationRes["matrix"],
                           "distortion" : calibrationRes["distortion"],
                           "newCameraMatrix" : calibrationRes["newCameraMatrix"],
                           "roi" : calibrationRes["roi"]}
    return test_calib_file_res

"""
    Прочитане на резултатите от stereo калбирането
"""
def readStereoTestCalib(dir,fname):
    # отваряне на файл за записване на резултата от калибрацията
    fs = cv.FileStorage(dir+fname, cv.FILE_STORAGE_READ)
    print(os.path.exists(dir+fname))
    # Параметри на камерата
    stereoCalibrationRes = {
        "cShape" : fs.getNode('cShape').mat().astype(int).flatten(),
        "pShape" : fs.getNode('pShape').mat().astype(int).flatten(),
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

# Прочита резултатът от калибрирането само за фунцията за getUndistortCalibrationRes,
# когато трябва да се използва стерео калибрирането
def readTestStereoCalibUndistor(stereoCalibrationRes):
    test_stereo_calib_file_res = {"matrix" : stereoCalibrationRes["cameraMatrix"],
                           "distortion" : stereoCalibrationRes["cameraDistortion"],
                           "newCameraMatrix" : None,
                           "roi" : stereoCalibrationRes["cRoi"]}
    return test_stereo_calib_file_res

# Изтрива изображенията създадени при тестването на камерата.
def delete_imgs(path):
    fileList = glob.glob(path)
    # Iterate over the list of filepaths & remove each file.
    for filePath in fileList:
        try:
            os.remove(filePath)
        except:
            print("Error while deleting file : ", filePath)

@pytest.fixture(scope="module")
def cam(request):
    # Подмяна на файла от калибрирането на камерата
    cameraOldName = CameraPi.CALIBRATION_DIR+CameraPi.CALIBRATION_FILE
    cameraNewName = CameraPi.CALIBRATION_DIR+CameraPi.CALIBRATION_FILE+"(1)"
    os.rename(cameraOldName, cameraNewName) # сменя се името на резултата от калибрирането, за да се сложи нов за тест
    shutil.copyfile(TEST_DIR+CameraPi.CALIBRATION_FILE, cameraOldName)

    # Подмяна на файла от стерео калибрирането
    stereoOldName = CameraPi.STEREO_CALIBRATION_DIR+CameraPi.STEREO_CALIBRATION_FILE
    stereoNewName = CameraPi.STEREO_CALIBRATION_DIR+CameraPi.STEREO_CALIBRATION_FILE+"(1)"
    os.rename(stereoOldName, stereoNewName) # сменя се името на резултата от калибрирането, за да се сложи нов за тест
    shutil.copyfile(TEST_DIR+CameraPi.STEREO_CALIBRATION_FILE, stereoOldName)

    yield CameraPi()

    # Възстановяване калибриране камера
    delete_imgs('{0}/*.png'.format(TEST_DIR))
    delete_imgs('./*.png')
    os.remove(cameraOldName)
    os.rename(cameraNewName,cameraOldName) # възстановяване на файла за калибриране
    # Възстановяване стерео калибриране
    delete_imgs('{0}/*.png'.format(TEST_DIR))
    delete_imgs('./*.png')
    os.remove(stereoOldName)
    os.rename(stereoNewName,stereoOldName) # възстановяване на файла за калибриране

# fixture, който съдържа стойностите за тест с flag, определящ как да бъде прочетено изображението(цветно, черно-бяло)
@pytest.fixture(params=[(None),
                         (cv.IMREAD_COLOR),
                         (cv.IMREAD_GRAYSCALE)])
def flag(request):
    return request.param

# fixture, който съдържа стойностите за тест с различни директории
@pytest.fixture(params=[(StructuredLight.SCAN_DIR)])
def path(request):
    return request.param

# fixture, който съдържа стойностите за тест с различни директории, които са невалидни
@pytest.fixture(params=[(""),
                        ("/"),
                        ("."),
                        ("./"),
                        ("./tests/test_files/projector/6.png")])
def invalidPath(request):
    return request.param

# fixture, който съдържа стойностите за тест с различни типове шаблони
@pytest.fixture(params=[(Patterns.IMAGE_PATTERN),
                         (Patterns.INV_PATTERN),
                         (Patterns.TRANS_PATTERN),
                         (Patterns.TRANS_INV_PATTERN)])
def patternCode(request):
    return request.param

# fixture, който съдържа стойностите за тест с различни стъпка на сканиране
@pytest.fixture(params=[(0),
                         (10),
                         (20),
                         (190)])
def scan_no(request):
    return request.param

# fixture, който съдържа стойностите за тест с различни номера на изображения
@pytest.fixture(params=[(0),
                         (1),
                         (7),
                         ('?'),
                         (None)])
def img_no(request):
    return request.param

# fixture, който съдържа стойностите за тест с различни директории
@pytest.fixture(params=[("./tests/test_files/camera/writeCalibResult")])
def calibrationDir(request):
    return request.param

#######################################################################################################################
## тестване заснемане на изображение
@pytest.mark.skipif(sys.platform == "win32", reason="does not run on windows")
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("dir,imageInd",
                        [("./tests/test_files/camera",""),
                         ("./tests/test_files/camera",1),
                         ("./tests/test_files/camera","Inv1"),
                         ("./tests/test_files/camera", None),
                         ("./tests/test_files/camera",-5),
                         ("./",-5)])
def test_takePhoto(cam,dir,imageInd):
    cam.takePhoto(dir,imageInd)
    assert (cv.imread('{0}/image{1}.jpg'.format(dir,imageInd), cv.IMREAD_COLOR) != None).all()

@pytest.mark.skipif(sys.platform == "win32", reason="does not run on windows")
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("dir,imageInd",
                        [("./asdf",""),
                         ("asdf",1),
                         ("/asdf","Inv1"),
                         (None, None),
                         (None,""),
                         (None,1),
                         ("",-5)]) #Липсват права в root
def test_takePhoto_error(cam,dir,imageInd):
    with pytest.raises(Exception):
        x=cam.takePhoto(dir,imageInd)
        print(x)
        print(cv.imread('{0}/image{1}.jpg'.format(dir,imageInd), cv.IMREAD_COLOR))

#######################################################################################################################
## тестване зареждане на правилен резултат от калибриране
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("dir,res",
                        [(Projector.CALIBRATION_DIR,
                            readTestCalibUndistor(readTestCalib(TEST_DIR,CameraPi.CALIBRATION_FILE))),
                         (CameraPi.CALIBRATION_DIR,None),
                         (CameraPi.STEREO_CALIBRATION_DIR,None),
                         ("",
                            readTestStereoCalibUndistor(readStereoTestCalib(TEST_DIR,CameraPi.STEREO_CALIBRATION_FILE))),
                         ("./",
                            readTestStereoCalibUndistor(readStereoTestCalib(TEST_DIR,CameraPi.STEREO_CALIBRATION_FILE))),
                         (StructuredLight.SCAN_DIR,
                            readTestStereoCalibUndistor(readStereoTestCalib(TEST_DIR,CameraPi.STEREO_CALIBRATION_FILE)))])
def test_getUndistortCalibrationRes(cam,dir,res):
    funcRes = cam.getUndistortCalibrationRes(dir)
    np.testing.assert_equal(funcRes,res)

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("dir",
                        [(None)])
def test_getUndistortCalibrationRes_error(cam,dir):
    with pytest.raises(Exception):
        cam.getUndistortCalibrationRes(dir)

#######################################################################################################################
## тестване зареждане на изображение - 1.png е черно бяло изображение, 3.png е цветно
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("fname",
                        [("./tests/test_files/projector/1.png"),
                         ("./tests/test_files/projector/3.png")])
def test_loadImage(cam,fname,flag):
    if flag is None:
        funcRes = cam.loadImage(fname)
        res = cv.imread(fname, cv.IMREAD_COLOR)
    else:
        funcRes = cam.loadImage(fname,flag)
        res = cv.imread(fname, flag)
    assert (funcRes == res).all()

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_loadImage_invalid(cam,invalidPath,flag):
    if flag == None:
        funcRes = cam.loadImage(invalidPath)
    else:
        funcRes = cam.loadImage(invalidPath,flag)
    assert funcRes == None

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("fname",
                        [(None)])
def test_loadImage_error(cam,fname,flag):
    with pytest.raises(Exception):
        if flag == None:
            cam.loadImage(fname)
        else:
            cam.loadImage(fname,flag)

######################################################################################################################
# тестване зареждане на изображение на шаблони
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_loadPatternImages(cam,path,patternCode,scan_no,flag,img_no):
    res = [] # масив със заредените изображения
    rd = flag
    if flag is None:
        funcRes = cam.loadPatternImages(path,patternCode,scan_no)
        imgsNames = natsorted(glob.glob('./{0}/image{1}{2}{3}.jpg'.format(path,scan_no,patternCode,'?')))
        rd = cv.IMREAD_GRAYSCALE
    elif img_no is None:
        funcRes = cam.loadPatternImages(path,patternCode,scan_no,flag)
        imgsNames = natsorted(glob.glob('./{0}/image{1}{2}{3}.jpg'.format(path,scan_no,patternCode,'?')))
    else:
        funcRes = cam.loadPatternImages(path,patternCode,scan_no,flag,img_no)
        imgsNames = natsorted(glob.glob('./{0}/image{1}{2}{3}.jpg'.format(path,scan_no,patternCode,img_no)))

    for fname in imgsNames:
        img = cam.loadImage(fname,rd)
        res.append(img)
    assert (funcRes == np.array(res)).all()

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_loadPatternImages_invalidPath(cam,invalidPath,patternCode,scan_no,flag,img_no):
    if flag is None:
        funcRes = cam.loadPatternImages(invalidPath,patternCode,scan_no)
    elif img_no is None:
        funcRes = cam.loadPatternImages(invalidPath,patternCode,scan_no,flag)
    else:
        funcRes = cam.loadPatternImages(invalidPath,patternCode,scan_no,flag,img_no)
    assert (funcRes == np.array([])).all()

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("fname",
                        [(None)])
def test_loadPatternImages_invalidPath2(cam,fname,patternCode,scan_no,flag,img_no):
    if flag is None:
        funcRes = cam.loadPatternImages(fname,patternCode,scan_no)
    elif img_no is None:
        funcRes = cam.loadPatternImages(fname,patternCode,scan_no,flag)
    else:
        funcRes = cam.loadPatternImages(fname,patternCode,scan_no,flag,img_no)
    assert (funcRes == np.array([])).all()

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("invalidPatternCode",
                        [(None),
                         (""),
                         ("asdf"),
                         (".")])
def test_loadPatternImages_invalidPatternCode(cam,path,invalidPatternCode,scan_no,flag,img_no):
    if flag is None:
        funcRes = cam.loadPatternImages(path,invalidPatternCode,scan_no)
    elif img_no is None:
        funcRes = cam.loadPatternImages(path,invalidPatternCode,scan_no,flag)
    else:
        funcRes = cam.loadPatternImages(path,invalidPatternCode,scan_no,flag,img_no)
    assert (funcRes == np.array([])).all()

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("invalidScan_no",
                        [(None),
                         (500),
                         (1000),
                         (-1),
                         (""),
                         (".")])
def test_loadPatternImages_invalidScan_no(cam,path,patternCode,invalidScan_no,flag,img_no):
    if flag is None:
        funcRes = cam.loadPatternImages(path,patternCode,invalidScan_no)
    elif img_no is None:
        funcRes = cam.loadPatternImages(path,patternCode,invalidScan_no,flag)
    else:
        funcRes = cam.loadPatternImages(path,patternCode,invalidScan_no,flag,img_no)
    assert (funcRes == np.array([])).all()


@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("invalidImg_no",
                        [(20),
                         (500),
                         (1000),
                         (-1),
                         (""),
                         (".")])
def test_loadPatternImages_invalidImg_no(cam,path,patternCode,scan_no,flag,invalidImg_no):
    funcRes = cam.loadPatternImages(path,patternCode,scan_no,flag,invalidImg_no)
    assert (funcRes == np.array([])).all()

#######################################################################################################################
## тестване записване на резултат от калибрирането

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("fname",
                        [("/CalibResult.json"),
                         ("/CalibResult(2).json"),
                         ("/CalibResult(3).json"),
                         ("/CalibResult(4).json")])
def test_writeCalibrationResult(cam,calibrationDir, fname):
    calibrationRes = readTestCalib("./tests/test_files/camera",fname)
    cam.writeCalibrationResult(calibrationDir, calibrationRes)
    res = readTestCalib(calibrationDir, CameraPi.CALIBRATION_FILE)
    expected = readTestCalib("./tests/test_files/camera",fname)
    np.testing.assert_equal(res,expected)

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("calibrationDir, calibrationRes",
                        [(None,readTestCalib("./tests/test_files/camera","/CalibResult.json")),
                         (calibrationDir,None)])
def test_writeCalibrationResult_error(cam,calibrationDir, calibrationRes):
    with pytest.raises(Exception):
        cam.writeCalibrationResult(calibrationDir, calibrationRes)

#######################################################################################################################
## тестване записване на резултат от стерео калибриране

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("fname",
                        [("/StereoCalibResult.json"),
                         ("/StereoCalibResult(2).json"),
                         ("/StereoCalibResult(3).json")])
def test_writeStereoCalibrationResult(cam,calibrationDir, fname):
    calibrationRes = readStereoTestCalib("./tests/test_files/camera",fname)
    cam.writeStereoCalibrationResult(calibrationRes)
    res = readStereoTestCalib(CameraPi.STEREO_CALIBRATION_DIR, CameraPi.STEREO_CALIBRATION_FILE)
    expected = readStereoTestCalib("./tests/test_files/camera",fname)
    np.testing.assert_equal(res,expected)

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("calibrationDir",
                        [(None)])
def test_writeStereoCalibrationResult_error(cam,calibrationDir):
    with pytest.raises(Exception):
        cam.writeStereoCalibrationResult(calibrationRes)

#######################################################################################################################
## тестване зареждане на резултат от калибриране

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("dir",
                        [("./tests/test_files/camera"),
                         (CameraPi.CALIBRATION_DIR),
                         (Projector.CALIBRATION_DIR)])
def test_readCalibrationResult(cam,dir):
    res = cam.readCalibrationResult(dir)
    expected = readTestCalib(dir,CameraPi.CALIBRATION_FILE)
    np.testing.assert_equal(res,expected)

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("calibrationDir",
                        [(None)])
def test_readCalibrationResult_error(cam,calibrationDir):
    with pytest.raises(Exception):
        cam.readCalibrationResult(calibrationDir)

#######################################################################################################################
## тестване зареждане на резултат от стерео калибриране

@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_readStereoCalibrationResult(cam):
    res = cam.readStereoCalibrationResult()
    expected = readStereoTestCalib(CameraPi.STEREO_CALIBRATION_DIR,CameraPi.STEREO_CALIBRATION_FILE)
    np.testing.assert_equal(res,expected)
