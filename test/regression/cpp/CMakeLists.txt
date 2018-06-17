##############################################################################
# Prep
##############################################################################

# Test binary name prefix, used by delphyne_build_tests
set(TEST_TYPE UNIT)

# ----------------------------------------
# Helper library.
add_library(test_helpers helpers.cc)

target_include_directories(
  test_helpers
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

target_link_libraries(test_helpers
  gtest
  gtest_main
  public_headers
  drake::drake
  ignition-msgs2::ignition-msgs2
  ignition-transport5::ignition-transport5
)

##############################################################################
# Tests
##############################################################################

set(SOURCES
  automotive_simulator_test.cc
  drake_driving_command_to_ign_translator_system_test.cc
  drake_simple_car_state_to_ign_translator_system_test.cc
  ign_driving_command_to_drake_translator_system_test.cc
  ign_model_v_to_lcm_viewer_draw_translator_system_test.cc
  ign_models_assembler_test.cc
  ign_publisher_system_test.cc
  ign_simple_car_state_to_drake_translator_system_test.cc
  ign_subscriber_system_test.cc
  interactive_simulation_stats_test.cc
  lcm_viewer_draw_to_ign_model_v_translator_system_test.cc
  lcm_viewer_load_robot_to_ign_model_v_translator_system_test.cc
  load_robot_aggregator_test.cc
  scene_system_test.cc
  simulation_run_stats_test.cc
  simulation_runner_test.cc
  time_conversion_test.cc
)

delphyne_build_tests(${SOURCES})
