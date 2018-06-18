##############################################################################
# Find Protobuf & Custom Macros
##############################################################################

include(${PROJECT_SOURCE_DIR}/cmake/Protobuf.cmake)

##############################################################################
# Generate Messages
##############################################################################

set(PROTO_MESSAGES
  automotive_driving_command.proto
  scene_request.proto
  simple_car_state.proto
  simulation_in_message.proto
  viewer_command.proto
  viewer2_comms.proto
)

protobuf_generate_cpp_with_descriptor(
  PROTO_SOURCES
  PROTO_HEADERS
  PROTO_DESCRIPTORS
  IMPORT_DIRS
    ${IGNITION-MSGS_INCLUDE_DIRS}
  DESTINATION
    ${PROJECT_BINARY_DIR}/include/delphyne/protobuf
  ${PROTO_MESSAGES}
)

##############################################################################
# Library
##############################################################################

# TODO: switch from PROTOBUF_INCLUDE_DIRS / PROTOBUF_LIBRARIES to
#       protobuf::libprotobuf (currently creates a pthread_create problem)

add_library(protobuf_messages ${PROTO_SOURCES})
set_target_properties(protobuf_messages PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-protobuf-messages)
target_include_directories(
  protobuf_messages
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
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

install(
  FILES
    ${PROTO_HEADERS}
    ${PROTO_DESCRIPTORS}
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/protobuf
)

install(
  FILES
    ${PROTO_MESSAGES}
  DESTINATION
    ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/proto
)

install(TARGETS protobuf_messages EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
