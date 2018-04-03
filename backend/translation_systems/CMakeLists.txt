add_library(drake_ign_translation_systems
  STATIC
  drake_driving_command_to_ign.cc
  drake_simple_car_state_to_ign.cc
  lcm_viewer_draw_to_ign_model_v.cc
  ign_driving_command_to_drake.cc
  ign_model_v_to_lcm_viewer_draw.cc
  ign_simple_car_state_to_drake.cc
)

target_link_libraries(drake_ign_translation_systems
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  protobuf
)

target_include_directories(drake_ign_translation_systems PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
