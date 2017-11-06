include (${project_cmake_dir}/Utils.cmake)

set (sources
  ign_to_lcm_translation.cc
  lcm_to_ign_translation.cc
  repeater_factory.cc
  repeater_manager.cc
  service_to_channel_translation.cc
  protobuf/headers/automotive_driving_command.pb.cc
  protobuf/headers/simple_car_state.pb.cc
  protobuf/headers/viewer2_comms.pb.cc
  protobuf/headers/viewer_command.pb.cc
)

set (gtest_sources
  ign_service_converter_TEST.cc
  ign_to_lcm_translation_TEST.cc
  lcm_channel_repeater_TEST.cc
  lcm_to_ign_translation_TEST.cc
  repeater_factory_TEST.cc
  repeater_manager_TEST.cc
  service_to_channel_translation_TEST.cc
)


find_dependency(robotlocomotion-lcmtypes)

# ----------------------------------------
# Create duplex-ign-lcm-bridge binary.
add_executable(duplex-ign-lcm-bridge
  duplex_ign_lcm_bridge.cc
  ${sources}
)
target_link_libraries(duplex-ign-lcm-bridge
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  ${PROTOBUF_PROTOC_LIBRARY}
  ${drake_LIBRARIES}
  ${lcm_LIBRARIES}
  drake::drake-lcmtypes-cpp
  robotlocomotion-lcmtypes::robotlocomotion-lcmtypes-cpp
)
install(TARGETS duplex-ign-lcm-bridge DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
# ----------------------------------------


# ----------------------------------------
# Create lcm-mock-robot-publisher binary.
add_executable(lcm-mock-robot-publisher
  lcm_mock_robot_publisher.cc
  ${sources}
)
target_link_libraries(lcm-mock-robot-publisher
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  ${PROTOBUF_PROTOC_LIBRARY}
  ${drake_LIBRARIES}
  ${lcm_LIBRARIES}
  drake::drake-lcmtypes-cpp
  robotlocomotion-lcmtypes::robotlocomotion-lcmtypes-cpp
)
install(TARGETS lcm-mock-robot-publisher DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
# ----------------------------------------


# ----------------------------------------
# Tests
delphyne_build_tests(${gtest_sources})
# ----------------------------------------
