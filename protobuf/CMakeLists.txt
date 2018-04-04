##############################################################################
# Find Protobuf
##############################################################################

find_package(Protobuf REQUIRED)

##############################################################################
# Generate Messages
##############################################################################

set(MESSAGES
  automotive_driving_command.proto
  robot_model_request.proto
  simple_car_state.proto
  simulation_in_message.proto
  viewer_command.proto
  viewer2_comms.proto
)

set(PROTOBUF_IMPORT_DIRS ${IGNITION-MSGS_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/protobuf)
protobuf_generate_cpp(SOURCES PROTO_HEADER ${MESSAGES})

##############################################################################
# Library
##############################################################################

add_library(protobuf_messages ${SOURCES})
set_target_properties(protobuf_messages PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-protobuf-messages)
target_include_directories(
  protobuf_messages
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}${PROJECT_MAJOR_VERSION}>
    ${PROTOBUF_INCLUDE_DIRS}
)

target_link_libraries(
  protobuf_messages
  PUBLIC
    ${PROTOBUF_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
)

##############################################################################
# Install
##############################################################################

string(REPLACE "proto" "pb.h" HEADERS "${MESSAGES}")
string(REGEX REPLACE "([^;]+)" "${CMAKE_CURRENT_BINARY_DIR}/\\1" HEADERS "${HEADERS}")

delphyne_install_includes(
  ${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}/protobuf
  ${HEADERS}
)

install(TARGETS protobuf_messages EXPORT ${PROJECT_NAME}${PROJECT_MAJOR_VERSION}-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
