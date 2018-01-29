include (${project_cmake_dir}/Utils.cmake)

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
  ign_to_lcm_translation.cc
)

# ----------------------------------------
# Export lcm to ign translation as a library
add_library(delphyne_lcm_to_ign STATIC ${lcm_to_ign_sources})

target_link_libraries(delphyne_lcm_to_ign
  ${IGNITION-MSGS_LIBRARIES}
  ${lcm_LIBRARIES}
  ${drake_LIBRARIES}
  delphyne_protobuf_msgs
)

target_include_directories(delphyne_lcm_to_ign PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

install(FILES launcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/launcher)
install(PROGRAMS demo_launcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# ----------------------------------------
# Tests
delphyne_build_tests(${gtest_sources})
