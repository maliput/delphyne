##############################################################################
# Ignition Translation Systems
##############################################################################

add_library(translations
  agent_state_v_splitter.cc
  drake_driving_command_to_ign.cc
  ign_driving_command_to_drake.cc
  ign_model_v_to_ign_pose_v.cc
  ign_model_v_to_lcm_viewer_draw.cc
  lcm_viewer_draw_to_ign_model_v.cc
  lcm_viewer_load_robot_to_ign_model_v.cc
  pose_bundle_to_agent_state_v.cc
  time_conversion.cc
)

add_library(delphyne::translations ALIAS translations)

set_target_properties(translations
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_translations
)

target_include_directories(
  translations
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

target_link_libraries(translations
  PUBLIC
    drake::drake
    delphyne::protobuf_messages
    delphyne::public_headers
    ignition-common4::ignition-common4
    ignition-msgs7::ignition-msgs7
    ignition-transport10::ignition-transport10
    maliput::common
    protobuf
)

install(
  TARGETS translations
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
