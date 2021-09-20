# Prefix test name
set(TEST_TYPE UNIT)

##############################################################################
# Tests
##############################################################################

set(PY_SOURCE
    agent_py_test.py
    simulation_runner_py_test.py
)

delphyne_build_python_tests(${PY_SOURCE})