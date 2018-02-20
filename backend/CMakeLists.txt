include (${project_cmake_dir}/Utils.cmake)
include_directories(${PYTHON_INCLUDE_DIRS})
include(${CMAKE_INSTALL_PREFIX}/share/cmake/pybind11/pybind11Tools.cmake)

# ----------------------------------------
# C++ tests.

set (gtest_sources
  agent_plugin_loader_test.cc
  automotive_simulator_test.cc
  discrete_value_to_ignition_message_converter_test.cc
  ign_publisher_system_test.cc
  ign_to_lcm_translation_test.cc
  lcm_to_ign_translation_test.cc
  lcm_viewer_draw_to_ignition_message_converter_test.cc
  linb_any_test.cc
  scene_system_test.cc
  simple_car_state_to_ignition_message_converter_test.cc
  simulation_runner_test.cc
)

add_subdirectory(test/agent_plugin)

delphyne_build_tests(${gtest_sources})

# ----------------------------------------
# Ign publisher system library

add_library(ign_publisher_system
  SHARED
  abstract_value_to_ignition_message_converter.h
  discrete_value_to_ignition_message_converter.h
  ign_publisher_system.h
  ignition_message_converter.h
  simple_car_state_to_ignition_message_converter.h
  system.h
)

target_link_libraries(ign_publisher_system
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  delphyne_lcm_to_ign
  delphyne_protobuf_msgs
  scene_system
)

target_include_directories(ign_publisher_system PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# ----------------------------------------
# Ign subscriber system library

add_library(ign_subscriber_system
  SHARED
  ign_subscriber_system.h
  ignition_message_converter.h
)

target_link_libraries(ign_subscriber_system
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  delphyne_protobuf_msgs
)

target_include_directories(ign_subscriber_system PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# ----------------------------------------
# Lcm to ign translation library

add_library(delphyne_lcm_to_ign
  STATIC
  lcm_to_ign_translation.cc
  ign_to_lcm_translation.cc
)

target_link_libraries(delphyne_lcm_to_ign
  ${IGNITION-MSGS_LIBRARIES}
  ${drake_LIBRARIES}
  ${lcm_LIBRARIES}
  delphyne_protobuf_msgs
)

target_include_directories(delphyne_lcm_to_ign PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# ----------------------------------------
# Simulator-runner library

add_library(simulation_runner
  STATIC
  simulation_runner.cc
  system.h
)

target_link_libraries(simulation_runner
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  delphyne_protobuf_msgs
  pybind11::module
)

target_include_directories(simulation_runner PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# ----------------------------------------
# Automotive-simulator library

add_library(automotive_simulator
  STATIC
  agent_plugin_loader.cc
  automotive_simulator.cc
  ign_subscriber_system.h
  linb-any
  system.h
)

target_link_libraries(automotive_simulator
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  delphyne_protobuf_msgs
  delphyne_lcm_to_ign
  ign_publisher_system
  ign_subscriber_system
  scene_system
)

target_include_directories(automotive_simulator PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# ----------------------------------------
# Scene-builder library

add_library(scene_system
  SHARED
  scene_builder_system.cc
  scene_system.cc
  system.h
)

target_link_libraries(scene_system
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  delphyne_lcm_to_ign
)

target_include_directories(scene_system PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
# ----------------------------------------
# Automotive demo
add_executable(automotive-demo
  automotive_demo.cc
  system.h
)

target_link_libraries(automotive-demo
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  delphyne_lcm_to_ign
  delphyne_protobuf_msgs
  pthread
  pybind11::module
  simulation_runner
  automotive_simulator
)

delphyne_install_includes(
  ${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}/backend
  agent_plugin_base.h
  linb-any
  system.h
)

# ----------------------------------------
# Test helper library.
add_library(test_helpers test/helpers.cc)

target_link_libraries(test_helpers
  gtest
  gtest_main
  ${drake_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
)

target_include_directories(test_helpers PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# ----------------------------------------
# Python bindings.

pybind11_add_module(simulation_runner_py simulation_runner_py.cc)

target_link_libraries(simulation_runner_py PRIVATE
  automotive_simulator
  simulation_runner
  ${drake_LIBRARIES}
  pybind11::module
  ${PYTHON_LIBRARIES}
)

# ----------------------------------------
# Installs libraries and binaries.

install(FILES delphyne_utils.py DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/utils)

install(TARGETS
  automotive_simulator
  scene_system
  simulation_runner
  simulation_runner_py
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(TARGETS automotive-demo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

install(PROGRAMS
  python_examples/keyboard_controlled_simulation.py
  python_examples/paused_mode_example.py
  python_examples/python_bindings_example.py
  python_examples/realtime_rate_changer.py
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# Python tests
install(PROGRAMS test/simulation_runner_py_test.py DESTINATION ${CMAKE_BINARY_DIR}/backend)

# Launcher
install(FILES launcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/launcher)
install(PROGRAMS demo_launcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
