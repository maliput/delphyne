include (${project_cmake_dir}/Utils.cmake)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})

# ----------------------------------------
# Prepare sources
set (sources
  simulation_runner.cc
)

set (gtest_sources
  simulation_runner_test.cc
)

# ----------------------------------------
# Automotive demo
add_executable(automotive-demo
  automotive_demo.cc
  ${sources}
)

target_link_libraries(automotive-demo
  delphyne_protobuf_msgs
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  pybind11::module
  ${PYTHON_LIBRARIES}
  pthread
)

install(TARGETS automotive-demo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# ----------------------------------------
# Tests
delphyne_build_tests(${gtest_sources})

# ----------------------------------------
# Build the simulation runner as a library.
add_library(simulation-runner
    STATIC
    simulation_runner.cc)

set_target_properties(simulation-runner
    PROPERTIES
    PUBLIC_HEADER
    simulation_runner.h)

target_link_libraries(simulation-runner
  delphyne_protobuf_msgs
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  pybind11::module
  ${PYTHON_LIBRARIES}
)

target_include_directories(simulation-runner PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# ----------------------------------------
# Python bindings

# Define the wrapper library for python
add_library(simulation_runner_py
  SHARED
  simulation_runner_py.cc)

# don't prepend wrapper library name with lib
set_target_properties(simulation_runner_py PROPERTIES PREFIX "")

target_link_libraries(simulation_runner_py
  simulation-runner
  delphyne_protobuf_msgs
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  pybind11::module
  ${PYTHON_LIBRARIES}
  )

install(TARGETS simulation_runner_py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(PROGRAMS python_bindings_example.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
