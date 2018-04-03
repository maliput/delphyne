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

add_library(protobuf ${SOURCES})
set_target_properties(protobuf PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-protobuf)
target_include_directories(
  protobuf
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}${PROJECT_MAJOR_VERSION}>
    ${PROTOBUF_INCLUDE_DIRS}
)

target_link_libraries(
  protobuf
  PUBLIC
    ${PROTOBUF_LIBRARIES}
    ${IGNITION-MSGS_LIBRARIES}
)

##############################################################################
# Install
##############################################################################

file(GLOB HEADERS ${CMAKE_CURRENT_BINARY_DIR}/*.pb.h)

delphyne_install_includes(
  ${PROJECT_NAME}${PROJECT_MAJOR_VERSION}/${PROJECT_NAME}/protobuf
  ${HEADERS}
)

install(TARGETS protobuf EXPORT ${PROJECT_NAME}${PROJECT_MAJOR_VERSION}-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
