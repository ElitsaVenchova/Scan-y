import pytest
import cv2 as cv
import numpy as np
import math

from structuredlight.structuredlight import Patterns

@pytest.fixture(scope="module")
def patt(request):
    return Patterns()
#######################################################################################################################
# fixture, който съдържа стойностите за тест с различни резолюции
@pytest.fixture(params=[(615,360),
                        (50,10),
                        (1280,720),
                        (1920,1080),
                        (2048,1152),
                        (0,0)])
def pSize(request):
    return request.param

# fixture, който съдържа стойностите за тест, които трябва да дадът грешка
@pytest.fixture(params=[(615,None),
                        (None,360),
                        (None,None)])
def pSize_error(request):
    return request.param
#######################################################################################################################
# fixture, който съдържа шаблони
@pytest.fixture(params=[(np.array([[0,0,0,0],[0,0,0,0]])),
                        (np.array([[255,255],[255,255]])),
                        (np.array([[0],[0]])),
                        (np.array([[0,255,0,255,0,255],[0,255,0,255,0,255]])),
                        (np.array([[],[]])),
                        (np.array([[255,255],[255,0]])),
                        (np.array([[255,255],[255,0]])),
                        (np.array([[0,255],[255,0],[0,255]])),
                        (np.array([]))])
def pattern(request):
    return request.param

# fixture, който съдържа стойностите за тест, които трябва да дадът грешка
@pytest.fixture(params=[(None),
                        (1),
                        (-1),
                        (0)])
def pattern_error(request):
    return request.param
#######################################################################################################################
# тестване дали стартирането и спирането на проекторът работи
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_white(patt,pSize):
    height, width = pSize
    pattWhite = patt.white(pSize)
    assert (pattWhite[patt.WHITE_PATTERN]==255*np.ones((1,height, width), np.uint8)).all()

# тестване дали стартирането и спирането на проекторът работи
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_white_error(patt,pSize_error):
    with pytest.raises(Exception):
        patt.white(pSize_error)

#######################################################################################################################
# тест на черния шаблон
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_black(patt,pSize):
    height, width = pSize
    pattBlack = patt.black(pSize)
    assert (pattBlack[patt.BLACK_PATTERN]==255*np.zeros((1,height, width), np.uint8)).all()

# тест на черния шаблон със стойности, за които трябва да даде грешка
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_black_error(patt,pSize_error):
    with pytest.raises(Exception):
        patt.black(pSize_error)
#######################################################################################################################
# тест на openCv.greyCode шаблон
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_openCVGray(patt,pSize):
    height, width = pSize
    pattopencvGray = patt.opencvGray(pSize)
    graycode = cv.structured_light_GrayCodePattern.create(width,height)
    assert ((len(pattopencvGray[patt.GRAY_CODE_PATTERN]) == 0
            and len(graycode.generate()[1])==0) or
        ((pattopencvGray[patt.GRAY_CODE_PATTERN][0]==graycode.generate()[1][0]).all() and
        (pattopencvGray[patt.GRAY_CODE_PATTERN][1]==graycode.generate()[1][1]).all()))
#######################################################################################################################
# тест на определянето на броя шаблони за черно белите шаблони(binary и gray code)
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_binaryCodePattCnt(patt,pSize):
    height, width = pSize
    pattBinatyCodePattCnt = patt.binaryCodePattCnt(pSize)
    if pSize == (0,0):
        assert pattBinatyCodePattCnt==(0,0)
    else:
        assert pattBinatyCodePattCnt==(int(math.log2(width)),int(math.log2(height)))

# тест на определянето на броя шаблони за черно белите шаблони(binary и gray code), за които трябва да даде грешка
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_binaryCodePattCnt_error(patt,pSize_error):
    with pytest.raises(Exception):
        patt.binaryCodePattCnt(pSize_error)
#######################################################################################################################
# тест на добавянето на височина на шаблон, защото при генерирането се изчисляват само стойностите за реда.
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("imgMatr,height",
                        [(np.array([[0,0,0,0],[0,0,0,0]]),5),
                        (np.array([[255,255],[255,255]]),10),
                        (np.array([[0],[0]]),200),
                        (np.array([[0,255,0,255,0,255],[0,255,0,255,0,255]]),1),
                        (np.array([[],[]]),10),
                        (np.array([[255,255],[255,0]]),10),
                        (np.array([[255,255],[255,0]]),0),
                        (np.array([[1,2],[3,4],[5,6]]),100)])
def test_addHeight(patt,imgMatr, height):
    x, y = imgMatr.shape
    pattAddHeight = patt.addHeight(imgMatr, height)
    assert (pattAddHeight == np.reshape(np.tile(imgMatr, height),(x,height,y))).all()

# тест на добавянето на височина на шаблон, за които трябва да даде грешка
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("imgMatr,height",
                        [(np.array([]),10),
                         (np.array([[255,255],[255,0]]),None),
                         (None,10)])
def test_addHeight_error(patt,imgMatr, height):
    with pytest.raises(Exception):
        patt.addHeight(imgMatr, height)
#######################################################################################################################
# тест на обръщане на черно-бял шаблон. Черното става бяла и обратното
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_invert(patt,pattern):
    pattInvert = patt.invert(pattern)
    assert (pattInvert == np.array([255-img for img in pattern])).all()

# тест на обръщане на черно-бял шаблон, за които трябва да даде грешка
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_invert_error(patt,pattern_error):
    with pytest.raises(Exception):
        patt.invert(pattern_error)
#######################################################################################################################
# тест на обръщане на черно-бял шаблон. Черното става бяла и обратното
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_transpose(patt,pattern):
    pattInvert = patt.transpose(pattern)
    assert (pattInvert == np.array([ img.T for img in pattern])).all()

# тест на обръщане на черно-бял шаблон, за които трябва да даде грешка
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_transpose_error(patt,pattern_error):
    with pytest.raises(Exception):
        patt.transpose(pattern)
