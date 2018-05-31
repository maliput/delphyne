##############################################################################
# Ignition Translation Systems
##############################################################################

add_library(translations
  drake_driving_command_to_ign.cc
  drake_simple_car_state_to_ign.cc
  lcm_viewer_draw_to_ign_model_v.cc
  lcm_viewer_load_robot_to_ign_model_v.cc
  ign_driving_command_to_drake.cc
  ign_model_v_to_lcm_viewer_draw.cc
  ign_simple_car_state_to_drake.cc
)
set_target_properties(translations PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-translations)
target_link_libraries(translations
  PUBLIC
    protobuf
    protobuf_messages
    ${drake_LIBRARIES}
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
  PRIVATE
    time_conversion
)

install(TARGETS translations
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
