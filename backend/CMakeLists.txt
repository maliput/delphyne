include (${project_cmake_dir}/Utils.cmake)
include_directories(${PYTHON_INCLUDE_DIRS})
# ----------------------------------------
# Prepare sources
set (sources
  automotive_simulator.cc
  ign_publisher_system.cc
  scene_system.cc
  simulation_runner.cc
  ign_publisher_system.h
  input_port_to_ign_converter.h
  abstract_input_to_ign_converter.h
  vector_input_to_ign_converter.h
  simple_car_state_input_to_ign_converter.h
)

set (gtest_sources
  automotive_simulator_test.cc
  ign_publisher_system_test.cc
  scene_system_test.cc
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
  delphyne_lcm_to_ign
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  pybind11::module
  ${PYTHON_LIBRARIES}
)

install(TARGETS automotive-demo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# ----------------------------------------
# Tests
delphyne_build_tests(${gtest_sources})

# ----------------------------------------
# Python bindings

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
  pybind11::module
  ${PYTHON_LIBRARIES}
)

target_include_directories(simulation-support PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# ----------------------------------------
# Python bindings

# Define the wrapper library for python
add_library(simulation_runner_py
  SHARED
  ign_publisher_system.h
  simulation_runner_py.cc
  scene_system.cc)

# Don't prepend wrapper library name with lib
set_target_properties(simulation_runner_py PROPERTIES PREFIX "")

target_link_libraries(simulation_runner_py
  simulation-support
  delphyne_protobuf_msgs
  delphyne_lcm_to_ign
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  pybind11::module
  ${PYTHON_LIBRARIES}
  )

install(TARGETS simulation_runner_py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(FILES utils.py DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/utils)
install(PROGRAMS python_examples/python_bindings_example.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(PROGRAMS python_examples/keyboard_controlled_simulation.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(PROGRAMS python_examples/paused_mode_example.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
