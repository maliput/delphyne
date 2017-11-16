include (${project_cmake_dir}/Utils.cmake)

set (sources
  SimulationRunner.cc
)

set (gtest_sources
  SimulationRunner_TEST.cc
)

add_executable(automotive-demo
  automotive_demo.cc
  ${sources}
)

target_link_libraries(automotive-demo
  delphyne_protobuf_msgs
  ${drake_LIBRARIES}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)

install(TARGETS automotive-demo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# ----------------------------------------
# Tests
delphyne_build_tests(${gtest_sources})
# ----------------------------------------
