##############################################################################
# Ignition Translation Systems
##############################################################################

add_library(translations
  agent_state_v_splitter.cc
  drake_angular_rate_acceleration_command_to_ign.cc
  drake_driving_command_to_ign.cc
  ign_angular_rate_acceleration_command_to_drake.cc
  ign_driving_command_to_drake.cc
  ign_model_v_to_lcm_viewer_draw.cc
  lcm_viewer_draw_to_ign_model_v.cc
  lcm_viewer_load_robot_to_ign_model_v.cc
  pose_bundle_to_agent_state_v.cc
  time_conversion.cc
)
set_target_properties(translations PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-translations)
target_include_directories(
  translations
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

ament_target_dependencies(translations
  PUBLIC
    maliput
)

target_link_libraries(translations
  PUBLIC
    drake::drake
    ignition-common3::ignition-common3
    ignition-msgs2::ignition-msgs2
    ignition-transport5::ignition-transport5
    maliput::common
    protobuf
    protobuf_messages
    public_headers
)

install(TARGETS translations
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
