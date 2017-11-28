include (${project_cmake_dir}/Utils.cmake)

set (msgs
  automotive_driving_command.proto
  robot_model_request.proto
  simple_car_state.proto
  simulation_in_message.proto
  viewer_command.proto
  viewer2_comms.proto
)

set(PROTOBUF_IMPORT_DIRS ${IGNITION-MSGS_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER
  ${msgs}
)

add_library(delphyne_protobuf_msgs STATIC ${PROTO_SRC})

target_include_directories(delphyne_protobuf_msgs PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
