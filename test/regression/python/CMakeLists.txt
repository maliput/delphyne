# Prefix test name
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(PY_SOURCE
    simulation_runner_py_test.py)

delphyne_build_python_tests(${PY_SOURCE})