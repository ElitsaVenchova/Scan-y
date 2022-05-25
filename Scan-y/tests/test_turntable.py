import pytest

from structuredlight.structuredlight import Turntable

@pytest.fixture(params=["X","Y"],scope="module")
def init_clear(request):
    yield request.param
    # print("Clean")
    pass

@pytest.mark.final
@pytest.mark.parametrize("a,b,expected",
                        [("A","B","AB"),
                        ("C","D","CD")])
def test_step(a,b,expected,init_clear):
    print(a+init_clear)
    tr = Turntable()
    assert True

@pytest.mark.final
@pytest.mark.regression
def test_step2(init_clear):
    tr = Turntable()
    assert True

@pytest.mark.regression
def test_step3():
    tr = Turntable()
    assert True

def test_step4():
    tr = Turntable()
    assert True

@pytest.mark.skip
def test_step5():
    tr = Turntable()
    assert True


@pytest.mark.skip
@pytest.mark.regression
def test_step6():
    tr = Turntable()
    assert True
