Tests with pytest python library.

Tests are by modules i.e. every classes in Scan-y.

There is pytest.ini file. In it, 'pythonpath' is set to be current dir. In this way we can ran command 'pytest' or 'py.test' in every dir in the project. Otherwise it doesn't work and the only way to start pytest is 'python -m pytest'.
Next line in pytest.ini is custom markers definition. In this way they are documented and there is no warnings in tests output.

Tests are marked with markers which classify them like Regression, Final, Manual and etc.
To run all tests: 'pytest'
To run custom mark test: 'pytest -m <custom_mark>' like 'pytest -m final'
@pytest.mark.skip - inbuilt mark in pytest. If it is used, test is skipped.

Notes:
------------------------------
pytest -s -> Show stdout
------------------------------
@pitest.fixture
def setUp...
  и след това
def test_tt(setUp) - извиква горната фунция преди да се изпълни тази на тялото. Създава контекст за изпълнението на теста.
------------------------------
@pitest.fixture(scope="module")
def setUpModule...
  и след това
def test_tt(setUpModule) - задава контекст за всички тестове във файла. Функцията се изпълява при първия тест.
------------------------------
@pytest.fixture(params=["X","Y"],scope="module")
def init_clear(request):
    yield request.param - Параметризиране на създаването на контекст. Контекстът се създава два пъти и съответно всеки тест се изпълнява два пъти(тези, които използва върната стойност). 
------------------------------
@pitest.fixture(scope="module")
def setUpModule():
  db = MySql()
  yield db
  db.close()
    и след това
def test_tt(setUpModule) - задава контекст за всички тестове във файла. При първото извикване се изпълнява до 'yield db'. След приключване на последния тест се изпълняват редовете след 'yield db'. Разбира се може да се използва без 'scope="module"', тогава ще създава и изтрива контества след всеки тест.
------------------------------
@pytest.mark.parametrize("a,b,expected",
                        [("A","B","AB"),
                        ("C","D","CD")])
def test_step(a,b,expected): - задаване на параметри, които да бъдат тествани. Изпълнява се за всеки параметът. Контекстната променлива може да се сложи също в параметрите. Важни са имената, не реда.
