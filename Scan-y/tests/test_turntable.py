import pytest

from structuredlight.structuredlight import Turntable

# Създаване на обект за въртящата платформа
@pytest.fixture(scope="module")
def turntbl(request):
    return Turntable()

# Проверка на различни ротирания на въртящата платформа.
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("step_size,dir,cnt",
                        [(1,Turntable.CW,1),
                        (200,Turntable.CW,1),
                        (200,Turntable.CW,5),
                        (5,Turntable.CW,5),
                        (1,Turntable.CW,0),
                        (0,Turntable.CW,1),
                        (1,Turntable.CCW,1),
                        (200,Turntable.CCW,1),
                        (200,Turntable.CCW,5),
                        (5,Turntable.CCW,5),
                        (1,Turntable.CCW,0),
                        (0,Turntable.CCW,1)])
def test_step(turntbl, step_size, dir, cnt):
    #Няма assert. Ако гръмне, ще е неуспешен тест.
    turntbl.step(step_size, dir, cnt)

# Проверка на различни стойнисти, при които се очаква функцията да върне грешка.
@pytest.mark.final
@pytest.mark.regression
@pytest.mark.unit
@pytest.mark.parametrize("step_size,dir,cnt",
                        [(None,Turntable.CW,1),
                        (None,Turntable.CW,200),
                        (1,Turntable.CW,None),
                        (200,Turntable.CW,None),
                        (None,Turntable.CCW,1),
                        (None,Turntable.CCW,200),
                        (1,Turntable.CCW,None),
                        (200,Turntable.CCW,None)])
def test_step_error(turntbl, step_size, dir, cnt):
    with pytest.raises(Exception):
        turntbl.step(step_size, dir, cnt)
