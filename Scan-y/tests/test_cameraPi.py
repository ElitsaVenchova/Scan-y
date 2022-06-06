import pytest
import cv2 as cv
import glob
import os
import sys

from structuredlight.structuredlight import CameraPi

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
    yield CameraPi()
    delete_imgs('./tests/test_files/camera/*.png')
    delete_imgs('./*.png')

# fixture, който съдържа стойностите за тест с пътища
@pytest.fixture(params=[(),
                        ()])
def dir(request):
    return request.param

#######################################################################################################################
# тестване заснемане на изображение
@pytest.mark.skipif(sys.platform == "win32", reason="does not run on windows")
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("dir,imageInd",
                        [("./tests/test_files/camera",""),
                         ("./tests/test_files/camera",1),
                         ("./tests/test_files/camera","Inv1"),
                         ("./tests/test_files/camera", None),
                         ("./tests/test_files/camera",""),
                         ("./tests/test_files/camera",-5),
                         ("./",-5),
                         ("",-5)])
def test_takePhoto(cam,dir,imageInd):
    cam.takePhoto(dir,imageInd)
    assert cv.imread('{0}/image{1}.jpg'.format(dir,imageInd), cv.IMREAD_COLOR) != None

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
                         (None,1)])
def test_takePhoto_error(cam,dir,imageInd):
    with pytest.raises(Exception):
        cam.takePhoto(dir,imageInd)

#######################################################################################################################
## тестване заснемане на изображение
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("dir,imageInd",
                        [("./tests/test_files/camera",""),
                         ("./tests/test_files/camera",1),
                         ("./tests/test_files/camera","Inv1"),
                         ("./tests/test_files/camera", None),
                         ("./tests/test_files/camera",""),
                         ("./tests/test_files/camera",-5),
                         ("./",-5),
                         ("",-5)])
def test_takePhoto(cam,dir,imageInd):
    cam.takePhoto(dir,imageInd)
    assert cv.imread('{0}/image{1}.jpg'.format(dir,imageInd), cv.IMREAD_COLOR) != None
