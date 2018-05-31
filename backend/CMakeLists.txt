include (${project_cmake_dir}/Utils.cmake)
include_directories(${PYTHON_INCLUDE_DIRS})

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
# Real-time Clock and related aliases
add_library(realtime_utils delphyne_duration.h delphyne_realtime_clock.h delphyne_time_point.h)
set_target_properties(realtime_utils PROPERTIES LINKER_LANGUAGE CXX OUTPUT_NAME ${PROJECT_NAME}-realtime-clock)

install(TARGETS realtime_utils
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# ----------------------------------------
# Scene-builder library
#
add_library(scene_system
  load_robot_aggregator.cc
  scene_system.cc
  system.h
)
set_target_properties(scene_system PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-scene-system)
target_link_libraries(
  scene_system
  PUBLIC
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
  automotive_simulator.cc
  road_builder.h
  system.h
)
set_target_properties(automotive_simulator PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-automotive-simulator)
target_link_libraries(
  automotive_simulator
  PUBLIC
    public_headers
    scene_system
    protobuf_messages
    ${drake_LIBRARIES}
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
  PRIVATE
    drake_ign_translation_systems
    realtime_utils
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
  interactive_simulation_stats.cc
  simulation_run_stats.cc
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

delphyne_install_includes(
  ${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}/backend
  system.h
)

# ----------------------------------------
# Python bindings.
#
include(${PROJECT_SOURCE_DIR}/cmake/Python.cmake)
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
