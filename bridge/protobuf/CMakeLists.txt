include (${project_cmake_dir}/Utils.cmake)

set (msgs
  automotive_driving_command.proto
  simple_car_state.proto
  viewer_command.proto
  viewer2_comms.proto
)

set(PROTOBUF_IMPORT_DIRS /home/alexis/delphyne_ws/src/ign_msgs)
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER
  ${msgs}
)

add_custom_target(protobuf_compilation DEPENDS
  ${PROTO_SRC}￼
  ${PROTO_HEADER}
)

add_library(protobuf_msgs STATIC ${PROTO_SRC})

target_include_directories(protobuf_msgs PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
install(TARGETS protobuf_msgs
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
