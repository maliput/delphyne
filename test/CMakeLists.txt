##############################################################################
# Find Packages
##############################################################################

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

    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file} ${sources})

    target_include_directories(${BINARY_NAME}
      PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${PYTHON_INCLUDE_DIRS}
    )

    # Kind of an ugly catch-all bucket

    target_link_libraries(
      ${BINARY_NAME}
        agents
        common
        translations
        protobuf
        visualization
        drake::drake
        maliput::api
        multilane::multilane
        dragway::dragway
        ignition-common2::ignition-common2
        ignition-transport5::ignition-transport5
        ignition-msgs2::ignition-msgs2
        delphyne_test_utilities
        gtest_main
        gtest
        pthread
        backend
    )

    # Remove a warning in GTest.
    target_compile_options(${BINARY_NAME} PRIVATE "-Wno-sign-compare")

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
      --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)

    if (PYTHONINTERP_FOUND)
      # Check that the test produced a result and create a failure if
      # it didn't. Guards against crashed and timed out tests.
      add_test(check_${BINARY_NAME} python
        ${PROJECT_SOURCE_DIR}/test/utils/check_test_ran.py
        ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
    endif()
  endforeach()
endmacro()

macro(delphyne_build_python_tests)
  # Build all python tests
  foreach(PYTHON_TEST_file ${ARGN})
    string(REGEX REPLACE ".py" "" PY_TEST ${PYTHON_TEST_file})
    set(PY_TEST ${TEST_TYPE}_${PY_TEST})

    add_test(NAME ${PY_TEST} COMMAND ${PYTHON_EXECUTABLE} -m pytest ${CMAKE_CURRENT_SOURCE_DIR})
  endforeach()
endmacro()

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test/test_config.h)

##############################################################################
# Testing
##############################################################################

add_subdirectory(libgtest)
add_subdirectory(regression)


