##############################################################################
# Library
##############################################################################

include(${PROJECT_SOURCE_DIR}/cmake/Python.cmake)

add_library(backend
  agent_simulation.cc
  agent_simulation_builder.cc
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
set_target_properties(backend PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-backend)
target_include_directories(
  backend
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PYTHON_INCLUDE_DIRS}>
)

ament_target_dependencies(backend
  "drake"
  "maliput"
  "maliput-utilities"
)

target_link_libraries(
  backend
    protobuf_messages
    public_headers
    utility
    visualization
    drake::drake
    maliput::api
    maliput-utilities::maliput-utilities
    ignition-common3::ignition-common3
    ignition-transport5::core
    ignition-transport5::log
    ignition-msgs2::ignition-msgs2
    ${PYTHON_LIBRARIES}
    translations
)

install(TARGETS backend
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

target_link_libraries(replayer
  PRIVATE
    protobuf_messages
    public_headers
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
)

install(TARGETS replayer
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
