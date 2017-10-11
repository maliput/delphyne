include (${project_cmake_dir}/Utils.cmake)

# lcm_mock_robot_publisher
add_executable(lcm_mock_robot_publisher
  lcm_mock_robot_publisher.cc

  protobuf/headers/automotive_driving_command.pb.cc
  protobuf/headers/simple_car_state.pb.cc
  protobuf/headers/viewer_command.pb.cc
  protobuf/headers/viewer2_comms.pb.cc)

MESSAGE(STATUS "bridge")

target_link_libraries(lcm_mock_robot_publisher
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  lcm
)

# lcm-ign-transport-bridge
add_executable(lcm-ign-transport-bridge
  ign_to_lcm_translation.cc
  lcm_ign_transport_bridge.cc
  lcm_to_ign_translation.cc
  repeater_factory.cc
  repeater_manager.cc
  service_to_channel_translation.cc
  protobuf/headers/automotive_driving_command.pb.cc
  protobuf/headers/simple_car_state.pb.cc
  protobuf/headers/viewer_command.pb.cc
  protobuf/headers/viewer2_comms.pb.cc)

message(STATUS "lcm: ${INTERFACE_LINK_LIBRARIES}")

target_link_libraries(lcm-ign-transport-bridge
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  lcm
)

add_subdirectory(test)
