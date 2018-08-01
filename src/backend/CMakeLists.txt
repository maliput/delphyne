##############################################################################
# Library
##############################################################################

include(${PROJECT_SOURCE_DIR}/cmake/Python.cmake)

add_library(backend
  automotive_simulator.cc
  frame_pose_aggregator.cc
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
target_link_libraries(
  backend
  PUBLIC
    protobuf_messages
    public_headers
    utility
    drake::drake
    ignition-common2::ignition-common2
    ignition-transport5::core
    ignition-transport5::log
    ignition-msgs2::ignition-msgs2
    pybind11::module
    ${PYTHON_LIBRARIES}
  PRIVATE
    common
    translations
)

install(TARGETS backend
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
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)

install(TARGETS replayer
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
