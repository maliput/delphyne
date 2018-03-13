include (${project_cmake_dir}/Utils.cmake)
include_directories(${PYTHON_INCLUDE_DIRS})
include(${CMAKE_INSTALL_PREFIX}/share/cmake/pybind11/pybind11Tools.cmake)

# ----------------------------------------
# C++ tests.

set (gtest_sources
  agent_plugin_loader_test.cc
  automotive_simulator_test.cc
  ign_publisher_system_test.cc
  ign_driving_command_to_lcm_driving_command_translator_system_test.cc
  ign_model_v_to_lcm_viewer_draw_translator_system_test.cc
  ign_simple_car_state_to_lcm_simple_car_state_translator_system_test.cc
  lcm_to_ign_translation_test.cc
  lcm_driving_command_to_ign_driving_command_translator_system_test.cc
  lcm_simple_car_state_to_ign_simple_car_state_translator_system_test.cc
  lcm_viewer_draw_to_ign_model_v_translator_system_test.cc
  linb_any_test.cc
  scene_system_test.cc
  simulation_runner_test.cc
  time_conversion_test.cc
)

add_subdirectory(test/agent_plugin)

delphyne_build_tests(${gtest_sources})

# ----------------------------------------
# Lcm to ign translation library

add_library(delphyne_lcm_to_ign
  STATIC
  lcm_to_ign_translation.cc
)

target_link_libraries(delphyne_lcm_to_ign
  ${IGNITION-MSGS_LIBRARIES}
  ${drake_LIBRARIES}
  ${lcm_LIBRARIES}
  delphyne_protobuf_msgs
  time_conversion
)

target_include_directories(delphyne_lcm_to_ign PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# ----------------------------------------
# Lcm to ign translator system library
add_library(lcm_ign_translator_systems
  STATIC
  lcm_driving_command_to_ign_driving_command_translator_system.cc
  lcm_simple_car_state_to_ign_simple_car_state_translator_system.cc
  lcm_viewer_draw_to_ign_model_v_translator_system.cc
  ign_driving_command_to_lcm_driving_command_translator_system.cc
  ign_model_v_to_lcm_viewer_draw_translator_system.cc
  ign_simple_car_state_to_lcm_simple_car_state_translator_system.cc
)

target_link_libraries(lcm_ign_translator_systems
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  delphyne_protobuf_msgs
)

target_include_directories(lcm_ign_translator_systems PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

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
  linb-any
  road_builder.h
  system.h
)

target_link_libraries(automotive_simulator
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PYTHON_LIBRARIES}
  delphyne_protobuf_msgs
  delphyne_lcm_to_ign
  lcm_ign_translator_systems
  scene_system
  time_conversion
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
add_library(time_conversion
  time_conversion.cc
)

target_link_libraries(time_conversion)

target_include_directories(time_conversion PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

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

pybind11_add_module(python_bindings python_bindings.cc)

target_link_libraries(python_bindings PRIVATE
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
  python_bindings
  time_conversion
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(TARGETS automotive-demo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

install(PROGRAMS
  python_examples/keyboard_controlled_simulation.py
  python_examples/paused_mode_example.py
  python_examples/python_bindings_example.py
  python_examples/realtime_rate_changer.py
  python_examples/road_loading_example.py
  python_examples/time_bounded_simulation.py
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# Python tests
install(PROGRAMS test/simulation_runner_py_test.py DESTINATION ${CMAKE_BINARY_DIR}/backend)

# Launcher
install(FILES launcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/launcher)
install(PROGRAMS demo_launcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
