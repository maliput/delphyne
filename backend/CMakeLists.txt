include (${project_cmake_dir}/Utils.cmake)

set (sources
  automotive_simulator.cc
  simulation_runner.cc
)

set (gtest_sources
  simulation_runner_test.cc
  automotive_simulator_test.cc
)

add_executable(automotive-demo
  automotive_demo.cc
  ${sources}
)

target_link_libraries(automotive-demo
  delphyne_protobuf_msgs
  delphyne_lcm_to_ign
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)

install(TARGETS automotive-demo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# ----------------------------------------
# Tests
delphyne_build_tests(${gtest_sources})

# ----------------------------------------
# Python bindings

# Find necessary packages
find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(pybind11 REQUIRED)
include_directories(${pybind11_INCLUDE_DIRS})

# Build the simulation support for python bindings as a library.
add_library(simulation-support
    STATIC
    simulation_runner.cc
    automotive_simulator.cc)

target_link_libraries(simulation-support
  delphyne_protobuf_msgs
  delphyne_lcm_to_ign
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)

target_include_directories(simulation-support PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Define the wrapper library for python
add_library(simulation_runner_py
  SHARED
  simulation_runner_py.cc)

# Don't prepend wrapper library name with lib
set_target_properties(simulation_runner_py PROPERTIES PREFIX "")

target_link_libraries(simulation_runner_py
  simulation-support
  delphyne_protobuf_msgs
  delphyne_lcm_to_ign
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${pybind11_LIBRARIES}
  )

install(TARGETS simulation_runner_py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(PROGRAMS python_bindings_example.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
