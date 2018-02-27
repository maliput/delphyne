include (${project_cmake_dir}/Utils.cmake)

set (msgs
  automotive_driving_command.proto
  robot_model_request.proto
  simple_car_state.proto
  simulation_in_message.proto
  viewer_command.proto
  viewer2_comms.proto
)

set(PROTOBUF_IMPORT_DIRS ${IGNITION-MSGS_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/protobuf)
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER
  ${msgs}
)

# The loadable module support needs to have some symbols from the protobuf code
# visible so that the symbols are in the "dynamic" symbol table, instead of the
# normal symbol table.  Additionally, these symbols need to be public, but our
# default is '-fvisibility=hidden'.  The calls to set_source_files_properties
# make it so that visibility of the symbols from the protobuf files (and only
# the listed protobuf files) are visible.
set_source_files_properties(automotive_driving_command.pb.cc PROPERTIES COMPILE_FLAGS -fvisibility=default)
set_source_files_properties(simple_car_state.pb.cc PROPERTIES COMPILE_FLAGS -fvisibility=default)
add_library(delphyne_protobuf_msgs STATIC ${PROTO_SRC})

target_include_directories(delphyne_protobuf_msgs PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
