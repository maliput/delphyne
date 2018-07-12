##############################################################################
# Scene Builder
##############################################################################

add_library(scene_system
  load_robot_aggregator.cc
  scene_system.cc
)
set_target_properties(scene_system PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-scene-system)
target_include_directories(
  scene_system
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)
target_link_libraries(
  scene_system
  PUBLIC
    public_headers
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

##############################################################################
# Automotive Simulator
##############################################################################

# ----------------------------------------
# Automotive-simulator library
#
add_library(automotive_simulator
  automotive_simulator.cc
  frame_pose_aggregator.cc
  ign_models_assembler.cc
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
    scene_system
    protobuf_messages
    public_headers
    ${drake_LIBRARIES}
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
  PRIVATE
    translations
)

install(TARGETS automotive_simulator
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# ----------------------------------------
# Simulator-runner library
#
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
    ${IGNITION-COMMON_LIBRARIES}
    ignition-transport5::core
    ignition-transport5::log
    ${IGNITION-MSGS_LIBRARIES}
    drake::drake
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(TARGETS simulation_runner
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
