##############################################################################
# Automotive Simulator
##############################################################################

add_library(automotive_simulator
  automotive_simulator.cc
  frame_pose_aggregator.cc
  ign_models_assembler.cc
  load_robot_aggregator.cc
  scene_system.cc
)
set_target_properties(automotive_simulator PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-automotive-simulator)
target_include_directories(
  automotive_simulator
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)
target_link_libraries(
  automotive_simulator
  PUBLIC
    protobuf_messages
    public_headers
    drake::drake
    ignition-common2::ignition-common2
    ignition-transport5::ignition-transport5
    ignition-msgs2::ignition-msgs2
  PRIVATE
    translations
)

install(TARGETS automotive_simulator
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

##############################################################################
# Simulation Runner
##############################################################################

include(${PROJECT_SOURCE_DIR}/cmake/Python.cmake)

add_library(simulation_runner
  interactive_simulation_stats.cc
  simulation_run_stats.cc
  simulation_runner.cc
)
set_target_properties(simulation_runner PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-simulation-runner)
target_include_directories(
  simulation_runner
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PYTHON_INCLUDE_DIRS}>
)
target_link_libraries(
  simulation_runner
  PUBLIC
    automotive_simulator
    protobuf_messages
    public_headers
    ignition-common2::ignition-common2
    ignition-transport5::core
    ignition-transport5::log
    ignition-msgs2::ignition-msgs2
    drake::drake
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(TARGETS simulation_runner
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

##############################################################################
# Replayer
##############################################################################

add_executable(replayer
  replayer.cc
)

target_link_libraries(replayer
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)

install(TARGETS replayer
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
