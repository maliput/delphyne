include (${project_cmake_dir}/Utils.cmake)

set (msgs
  automotive_driving_command.proto
  simple_car_state.proto
  viewer_command.proto
  viewer2_comms.proto
)

set(PROTOBUF_PROTOC_EXECUTABLE /usr/bin/protoc)

foreach(FIL ${msgs})
  get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
  get_filename_component(FIL_WE ${FIL} NAME_WE)

  list(APPEND PROTO_SRCS
    "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc")
  list(APPEND PROTO_HDRS
    "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT
      ${FIL_WE}.pb.cc
      ${FIL_WE}.pb.h
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS
      --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
      --proto_path=${CMAKE_CURRENT_SOURCE_DIR}:${PROJECT_SOURCE_DIR}/../ign_msgs
      ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${FIL}"
    VERBATIM
  )

endforeach()

add_library(protobuf_msgs STATIC ${PROTO_SRCS}) 

target_include_directories(protobuf_msgs PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS protobuf_msgs
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
