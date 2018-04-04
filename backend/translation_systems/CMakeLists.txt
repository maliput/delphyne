##############################################################################
# Ignition Translation Systems
##############################################################################

add_library(drake_ign_translation_systems
  drake_driving_command_to_ign.cc
  drake_simple_car_state_to_ign.cc
  lcm_viewer_draw_to_ign_model_v.cc
  ign_driving_command_to_drake.cc
  ign_model_v_to_lcm_viewer_draw.cc
  ign_simple_car_state_to_drake.cc
)
set_target_properties(drake_ign_translation_systems PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-drake-ign-translation-systems)
target_link_libraries(drake_ign_translation_systems
  PUBLIC
    protobuf
    ${drake_LIBRARIES}
    ${IGNITION-COMMON_LIBRARIES}
    ${IGNITION-TRANSPORT_LIBRARIES}
)

install(TARGETS drake_ign_translation_systems
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
