##############################################################################
# Find Packages
##############################################################################

find_package(ament_cmake_gtest REQUIRED)

# Find the Python interpreter for running the check_test_ran.py script
find_package(PythonInterp QUIET)

##############################################################################
# Macros
##############################################################################

macro(delphyne_build_tests)
  # Build all cpp tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})

    ament_add_gtest(${BINARY_NAME} ${GTEST_SOURCE_file} TIMEOUT 240)

    target_include_directories(${BINARY_NAME}
      PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${PYTHON_INCLUDE_DIRS}
    )

    # Kind of an ugly catch-all bucket
    target_link_libraries(
      ${BINARY_NAME}
        drake::drake
        delphyne::agents
        delphyne::backend
        delphyne::translations
        delphyne::utility
        delphyne::visualization
        delphyne::test_utilities
        ignition-common3::ignition-common3
        ignition-msgs5::ignition-msgs5
        ignition-transport8::ignition-transport8
        maliput::api
        maliput::common
        pthread
        protobuf
    )

    # Remove a warning in GTest.
    target_compile_options(${BINARY_NAME} PRIVATE "-Wno-sign-compare")

    if (PYTHONINTERP_FOUND)
      # Check that the test produced a result and create a failure if
      # it didn't. Guards against crashed and timed out tests.
      add_test(check_${BINARY_NAME} ${PYTHON_EXECUTABLE}
        ${PROJECT_SOURCE_DIR}/test/utils/check_test_ran.py
        ${CMAKE_BINARY_DIR}/test_results/delphyne/${BINARY_NAME}.gtest.xml)
    endif()
  endforeach()
endmacro()

macro(delphyne_build_python_tests)
# When sanitizers are activated python scripts are disabled.
  if (NOT ${SANITIZERS})
    # Build all python tests
    foreach(PYTHON_TEST_file ${ARGN})
      string(REGEX REPLACE ".py" "" PY_TEST ${PYTHON_TEST_file})
      set(PY_TEST ${TEST_TYPE}_${PY_TEST})

      add_test(NAME ${PY_TEST} COMMAND ${PYTHON_EXECUTABLE} -m pytest ${CMAKE_CURRENT_SOURCE_DIR})
    endforeach()
  endif()
endmacro()

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test/test_config.h)

##############################################################################
# Testing
##############################################################################

add_subdirectory(regression)


