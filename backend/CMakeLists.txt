include (${project_cmake_dir}/Utils.cmake)
include_directories(${PYTHON_INCLUDE_DIRS})
# ----------------------------------------
# Prepare sources
set (sources
  abstract_value_to_ignition_message_converter.h
  agent_plugin_loader.cc
  automotive_simulator.cc
  discrete_value_to_ignition_message_converter.h
  ign_publisher_system.h
  ign_subscriber_system.h
  ignition_message_converter.h
  scene_builder_system.cc
  scene_system.cc
  simple_car_state_to_ignition_message_converter.h
  simulation_runner.cc
  system.h
)

set (gtest_sources
  abstract_value_to_ignition_message_converter_test.cc
  agent_plugin_loader_test.cc
  automotive_simulator_test.cc
  discrete_value_to_ignition_message_converter_test.cc
  ign_publisher_system_test.cc
  linb_any_test.cc
  scene_system_test.cc
  simple_car_state_to_ignition_message_converter_test.cc
  simulation_runner_test.cc
)

add_subdirectory(test/agent_plugin)

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
  pthread
)

install(TARGETS automotive-demo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

delphyne_install_includes(
  ${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}/backend
  agent_plugin_base.h
  system.h
  linb-any
)

# ----------------------------------------
# Tests
delphyne_build_tests(${gtest_sources})

# Helpers
add_library(test_helpers
    test/helpers.cc)

target_link_libraries(test_helpers
  gtest
  gtest_main
  ${drake_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
)

target_include_directories(test_helpers PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
# ----------------------------------------
# Python bindings

# Build the simulation support for python bindings as a library.
add_library(simulation-support
    SHARED
    agent_plugin_loader.cc
    automotive_simulator.cc
    simulation_runner.cc
)

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
  ign_subscriber_system.h
  simulation_runner_py.cc
  scene_system.cc
  scene_builder_system.cc
)

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

# Python tests
install(PROGRAMS test/simulation_runner_py_test.py DESTINATION ${CMAKE_BINARY_DIR}/backend)
