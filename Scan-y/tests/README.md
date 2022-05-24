Tests with pytest python library.

Tests are by modules i.e. every classes in Scan-y.

There is pytest.ini file. In it, 'pythonpath' is set to be current dir. In this way we can ran command 'pytest' or 'py.test' in every dir in the project. Otherwise it doesn't work and the only way to start pytest is 'python -m pytest'.
Next line in pytest.ini is custom markers definition. In this way they are documented and there is no warnings in tests output.

Tests are marked with markers which classify them like Regression, Final, Manual and etc.
To run all tests: 'pytest'
To run custom mark test: 'pytest -m <custom_mark>' like 'pytest -m final'
@pytest.mark.skip - inbuilt mark in pytest. If it is used, test is skipped.
