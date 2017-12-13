##################################################
# Hack: to just build the files but not add them into the testsuite
# (like auxiliary apps using gtest files called from other tests)
# set the delphyne_SKIP_IN_TESTSUITE variable to true. The variable will
# be set to false at the end of the function.
macro (delphyne_build_tests)

  # Find the Python interpreter for running the
  # check_test_ran.py script
  find_package(PythonInterp QUIET)

  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    if(USE_LOW_MEMORY_TESTS)
      add_definitions(-DUSE_LOW_MEMORY_TESTS=1)
    endif(USE_LOW_MEMORY_TESTS)

    add_executable(${BINARY_NAME}
      test/${GTEST_SOURCE_file}
      ${sources}
    )

    add_dependencies(${BINARY_NAME}
      gtest gtest_main
    )

    target_link_libraries(${BINARY_NAME}
      ${Boost_LIBRARIES}
      ${drake_LIBRARIES}
      ${IGNITION-COMMON_LIBRARIES}
      ${IGNITION-MSGS_LIBRARIES}
      ${IGNITION-TRANSPORT_LIBRARIES}
      ${PROTOBUF_LIBRARY}
      ${PROTOBUF_PROTOC_LIBRARY}
      ${lcm_LIBRARIES}
      ${PYTHON_LIBRARIES}
      libgtest.a
      libgtest_main.a
      pthread
      delphyne_protobuf_msgs
    )

    # Remove a warning in GTest.
    target_compile_options(${BINARY_NAME} PRIVATE "-Wno-sign-compare")

    if (NOT DEFINED delphyne_SKIP_IN_TESTSUITE)
      set(delphyne_SKIP_IN_TESTSUITE False)
    endif()

    if (NOT delphyne_SKIP_IN_TESTSUITE)
      add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
        --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

      set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)

      if (PYTHONINTERP_FOUND)
        # Check that the test produced a result and create a failure if
        # it didn't. Guards against crashed and timed out tests.
        add_test(check_${BINARY_NAME} python
          ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
          ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
      endif()
    endif()
  endforeach()

  # If delphyne_SKIP_IN_TESTSUITE was set to True, back to default. This way
  # we request for explicit definition before calling the macro
  if (delphyne_SKIP_IN_TESTSUITE)
    set(delphyne_SKIP_IN_TESTSUITE False)
  endif()
endmacro()
