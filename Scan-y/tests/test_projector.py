import pytest
import cv2 as cv
import sys

from structuredlight.structuredlight import Projector
from structuredlight.structuredlight import CameraPi

@pytest.fixture(scope="module")
def proj(request):
    return Projector(CameraPi())
#######################################################################################################################
# тестване дали стартирането и спирането на проекторът работи
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_start_stop(proj):
    proj.start()
    proj.stop()
#######################################################################################################################
# fixture, който стартира прокетора, после изпълнява някакви тестове за визуализация и след това спира проектора.
@pytest.fixture
def start_stop(proj):
    proj.start()
    yield proj
    proj.stop()
# fixture, който съдържа стойностите за тест с различни изображения по цвят и различно число в тях
@pytest.fixture(params=[("./tests/test_files/projector/1.png"),
                         ("./tests/test_files/projector/2.png"),
                         ("./tests/test_files/projector/3.png"),
                         ("./tests/test_files/projector/4.png"),
                         ("./tests/test_files/projector/5.png")])
def path(request):
    return request.param

# fixture, който съдържа стойностите за тест, които трябва да дадът грешка
@pytest.fixture(params=[(None),
                         ("./tests/test_files/projector/6.png"),
                         ("./tests/test_files/projector/dada.png"),
                         ("4.png")])
def path_error(request):
    return request.param
#######################################################################################################################
# тестване показване на изображение по подаден път.
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_playImageByPath(start_stop,path):
    img = cv.imread(path)
    start_stop.playImageByPath(path)

# тестване показване на изображение по подаден път.
@pytest.mark.skipif(sys.platform == "win32", reason="does not run on windows")
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_playImageByPath_error(start_stop,path_error):
    with pytest.raises(Exception):
        start_stop.playImageByPath(path_error)
#######################################################################################################################
# тестване показване на изображение по подадено изображение.
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_playImage(start_stop,path):
    img = cv.imread(path)
    start_stop.playImage(img)

# тестване показване на изображение по подаден път.
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
def test_playImage_error(start_stop,path_error):
    img = cv.imread(path_error)
    with pytest.raises(Exception):
        start_stop.playImage(img)
