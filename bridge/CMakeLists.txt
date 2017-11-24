include (${project_cmake_dir}/Utils.cmake)

# ----------------------------------------
# Generate protobuf headers
add_subdirectory(protobuf)

set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER}
  PROPERTIES GENERATED TRUE)

# ----------------------------------------
# Set sources and gtest_sources
set (sources
  ign_to_lcm_translation.cc
  lcm_to_ign_translation.cc
  repeater_factory.cc
  repeater_manager.cc
  service_to_channel_translation.cc
)

set (gtest_sources
  ign_service_converter_test.cc
  ign_to_lcm_translation_test.cc
  lcm_channel_repeater_test.cc
  lcm_to_ign_translation_test.cc
  repeater_factory_test.cc
  repeater_manager_test.cc
  service_to_channel_translation_test.cc
)

set (lcm_to_ign_sources
  lcm_to_ign_translation.cc
)

# ----------------------------------------
# Export lcm to ign translation as a library
add_library(delphyne_lcm_to_ign STATIC ${lcm_to_ign_sources})

target_link_libraries(delphyne_lcm_to_ign
  ${IGNITION-MSGS_LIBRARIES}
  ${lcm_LIBRARIES}
  ${drake_LIBRARIES}
)

target_include_directories(delphyne_lcm_to_ign PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

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
  delphyne_protobuf_msgs
)

install(TARGETS duplex-ign-lcm-bridge DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

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
  delphyne_protobuf_msgs
)

install(TARGETS lcm-mock-robot-publisher DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(FILES launcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/launcher)
install(PROGRAMS mocked_robot_demo.py demo_launcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# ----------------------------------------
# Tests
delphyne_build_tests(${gtest_sources})
