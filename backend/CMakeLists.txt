include (${project_cmake_dir}/Utils.cmake)
include_directories(${PYTHON_INCLUDE_DIRS})
include(${CMAKE_INSTALL_PREFIX}/share/cmake/pybind11/pybind11Tools.cmake)

##############################################################################
# Common Libraries
##############################################################################

# ----------------------------------------
# Time Conversions (includes ignition conversions)
add_library(time_conversion time_conversion.cc)
set_target_properties(time_conversion PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-time-conversion)
target_link_libraries(
  time_conversion
  PUBLIC
    ${IGNITION-MSGS_LIBRARIES}
)
install(TARGETS time_conversion
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# ----------------------------------------
# Lcm to ign translation library
#
add_library(lcm_to_ignition_translation lcm_to_ign_translation.cc)
set_target_properties(lcm_to_ignition_translation PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-lcm-to-ignition-translation)
target_link_libraries(
  lcm_to_ignition_translation
  PUBLIC
    protobuf_messages
    ${IGNITION-MSGS_LIBRARIES}
    ${drake_LIBRARIES}
  PRIVATE
    time_conversion
)
install(TARGETS lcm_to_ignition_translation
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# ----------------------------------------
# Scene-builder library
#
add_library(scene_system
  scene_builder_system.cc
  scene_system.cc
  system.h
)
set_target_properties(scene_system PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-scene-system)
target_link_libraries(
  scene_system
  PUBLIC
    lcm_to_ignition_translation
    ${IGNITION-MSGS_LIBRARIES}
    ${drake_LIBRARIES}
  PRIVATE
    ${IGNITION-COMMON_LIBRARIES}
)
install(TARGETS scene_system
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

add_subdirectory(translation_systems)

##############################################################################
# Automotive Simulator
##############################################################################

# ----------------------------------------
# Automotive-simulator library
#
add_library(automotive_simulator
  agent_plugin_loader.cc
  automotive_simulator.cc
  linb-any
  road_builder.h
  system.h
)
set_target_properties(automotive_simulator PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-automotive-simulator)
target_link_libraries(
  automotive_simulator
  PUBLIC
    scene_system
    protobuf_messages
    ${drake_LIBRARIES}
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
  PRIVATE
    drake_ign_translation_systems
)

install(TARGETS automotive_simulator
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# ----------------------------------------
# Simulator-runner library
#
# Right now this can't be made shared - segfaults
# keyboard_controlled_simulation.py at the gil_scoped_acquire
# call
add_library(simulation_runner
  STATIC
  simulation_runner.cc
  system.h
)
# Position independent code can disappear once we are able to drop STATIC
set_property(TARGET simulation_runner PROPERTY POSITION_INDEPENDENT_CODE ON)
set_target_properties(simulation_runner PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-simulation-runner)
target_link_libraries(
  simulation_runner
  PUBLIC
    automotive_simulator
    protobuf_messages
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
    ${drake_LIBRARIES}
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(TARGETS simulation_runner
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# ----------------------------------------
# Automotive demo
#
add_executable(automotive-demo
  automotive_demo.cc
  system.h
)

target_link_libraries(automotive-demo
  simulation_runner
  automotive_simulator
  ${drake_LIBRARIES}
)

install(TARGETS automotive-demo RUNTIME DESTINATION bin)

delphyne_install_includes(
  ${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}/backend
  agent_plugin_base.h
  linb-any
  system.h
)

# ----------------------------------------
# Python bindings.
#
pybind11_add_module(python_bindings python_bindings.cc)

target_link_libraries(python_bindings
  PRIVATE
    simulation_runner
    automotive_simulator
    ${drake_LIBRARIES}
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(TARGETS python_bindings
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Creates an emtpy __init__.py file in order to enable python
# to identify `delphyne` as a module.
file(WRITE
  ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/delphyne/__init__.py "")

# ----------------------------------------
# Installs libraries and binaries.
#
install(
  FILES 
    python/bindings.py
    python/launcher.py
    python/simulation_utils.py 
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/delphyne)

install(PROGRAMS
  python/demo_launcher.py
  python/examples/keyboard_controlled_simulation.py
  python/examples/loadable_agent_simulation.py
  python/examples/realtime_rate_changer.py
  python/examples/road_loading.py
  python/examples/simple_python_binding.py
  python/examples/start_simulation_paused.py
  python/examples/time_bounded_simulation.py
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)

# Launcher
install(FILES python/launcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/launcher)
install(PROGRAMS python/demo_launcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

##############################################################################
# C++ tests.
##############################################################################

set (gtest_sources
  agent_plugin_loader_test.cc
  automotive_simulator_test.cc
  drake_driving_command_to_ign_translator_system_test.cc
  drake_simple_car_state_to_ign_translator_system_test.cc
  ign_driving_command_to_drake_translator_system_test.cc
  ign_model_v_to_lcm_viewer_draw_translator_system_test.cc
  ign_publisher_system_test.cc
  ign_simple_car_state_to_drake_translator_system_test.cc
  ign_subscriber_system_test.cc
  lcm_to_ign_translation_test.cc
  lcm_viewer_draw_to_ign_model_v_translator_system_test.cc
  linb_any_test.cc
  scene_system_test.cc
  simulation_runner_test.cc
  time_conversion_test.cc
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

add_subdirectory(test/agent_plugin)

delphyne_build_tests(${gtest_sources})
