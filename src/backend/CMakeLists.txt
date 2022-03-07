##############################################################################
# Library
##############################################################################

include(${PROJECT_SOURCE_DIR}/cmake/Python.cmake)

add_library(backend
  agent_simulation.cc
  agent_simulation_builder.cc
  fixed_phase_iteration_handler.cc
  frame_pose_aggregator.cc
  geometry_utilities.cc
  ign_models_assembler.cc
  data_logger.cc
  interactive_simulation_stats.cc
  load_robot_aggregator.cc
  scene_system.cc
  simulation_run_stats.cc
  simulation_runner.cc  # needs python
)

add_library(delphyne::backend ALIAS backend)

set_target_properties(backend
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_backend
)

target_include_directories(
  backend
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PYTHON_INCLUDE_DIRS}>
)

target_link_libraries(
  backend
    drake::drake
    delphyne::protobuf_messages
    delphyne::public_headers
    delphyne::translations
    delphyne::utility
    delphyne::visualization
    ignition-common3::ignition-common3
    ignition-transport8::core
    ignition-transport8::log
    ignition-msgs5::ignition-msgs5
    maliput::api
    maliput::base
    maliput::utilities
    ${PYTHON_LIBRARIES}
)

install(
  TARGETS backend
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

##############################################################################
# Log replayer
##############################################################################

add_executable(replayer
  replayer.cc
)

set_target_properties(replayer
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_replayer
)

target_link_libraries(replayer
  PRIVATE
    delphyne::protobuf_messages
    delphyne::public_headers
    ignition-common3::ignition-common3
    ignition-msgs5::ignition-msgs5
    ignition-transport8::core
    ignition-transport8::log
)

install(
  TARGETS replayer
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
