##############################################################################
# Find Packages
##############################################################################

# Find the Python interpreter for running the check_test_ran.py script
find_package(PythonInterp QUIET)

##############################################################################
# Macros
##############################################################################

macro(delphyne_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})

    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file} ${sources})

    target_include_directories(${BINARY_NAME}
      PRIVATE
        ${PYTHON_INCLUDE_DIRS}
    )

    # Kind of an ugly catch-all bucket
    target_link_libraries(
      ${BINARY_NAME}
      PRIVATE
        simulation_runner
        automotive_simulator
        scene_system
        drake_ign_translation_systems
        time_conversion
        protobuf
        ${drake_LIBRARIES}
        ${IGNITION-COMMON_LIBRARIES}
        ${IGNITION-TRANSPORT_LIBRARIES}
        ${IGNITION-MSGS_LIBRARIES}
        pybind11::module
        ${PYTHON_LIBRARIES}
        test_helpers
        gtest_main
        gtest
        pthread
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

##############################################################################
# Testing
##############################################################################

add_subdirectory(libgtest)
add_subdirectory(regression)

