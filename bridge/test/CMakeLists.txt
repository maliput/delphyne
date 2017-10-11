include (${project_cmake_dir}/Utils.cmake)

include_directories(
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}/
  ${CMAKE_BINARY_DIR}/test/
)

# lcm_channel_repeater_test
#delphyne_build_test(lcm_channel_repeater_test.cc
#  ../lcm_to_ign_translation.cc
#  #../protobuf/headers/viewer_command.pb.cc
#  #../protobuf/headers/viewer2_comms.pb.cc
#  #../protobuf/headers/simple_car_state.pb.cc
#)

# lcm_to_delphyne_translation_test
delphyne_build_test(lcm_to_ign_translation_test.cc
  ../lcm_to_ign_translation.cc
  ../protobuf/headers/viewer_command.pb.cc
  ../protobuf/headers/viewer2_comms.pb.cc
  ../protobuf/headers/simple_car_state.pb.cc
)
