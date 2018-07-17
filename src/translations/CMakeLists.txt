##############################################################################
# Ignition Translation Systems
##############################################################################

add_library(translations
  drake_driving_command_to_ign.cc
  drake_simple_car_state_to_ign.cc
  ign_driving_command_to_drake.cc
  ign_model_v_to_lcm_viewer_draw.cc
  ign_simple_car_state_to_drake.cc
  lcm_viewer_draw_to_ign_model_v.cc
  lcm_viewer_load_robot_to_ign_model_v.cc
  pose_bundle_to_simple_car_state_v.cc
  simple_car_state_v_splitter.cc
  time_conversion.cc
)
set_target_properties(translations PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-translations)
target_include_directories(
  translations
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)
target_link_libraries(translations
  PUBLIC
    public_headers
    protobuf
    protobuf_messages
    drake::drake
    ignition-common2::ignition-common2
    ignition-msgs2::ignition-msgs2
    ignition-transport5::ignition-transport5
)

install(TARGETS translations
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
