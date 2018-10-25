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
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/regression/cpp>
)

target_link_libraries(test_helpers
  gtest
  gtest_main
  public_headers
  drake::drake
  drake_gen
  ignition-common2::ignition-common2
  ignition-msgs2::ignition-msgs2
  ignition-transport5::ignition-transport5
)

##############################################################################
# Tests
##############################################################################

set(SOURCES
  agent_diagram_builder_test.cc
  automotive_simulator_test.cc
  calc_ongoing_road_position_test.cc
  calc_smooth_acceleration_test.cc
  car_vis_applicator_test.cc  
  curve2_test.cc  
  compression_test.cc
  data_logger_test.cc
  drake_driving_command_to_ign_translator_system_test.cc
  filesystem_test.cc
  frame_pose_aggregator_test.cc
  idm_controller_test.cc
  idm_planner_test.cc
  ign_driving_command_to_drake_translator_system_test.cc
  ign_model_v_to_lcm_viewer_draw_translator_system_test.cc
  ign_models_assembler_test.cc
  ign_publisher_system_test.cc
  ign_subscriber_system_test.cc
  interactive_simulation_stats_test.cc
  lcm_viewer_draw_to_ign_model_v_translator_system_test.cc
  lcm_viewer_load_robot_to_ign_model_v_translator_system_test.cc
  load_robot_aggregator_test.cc
  mobil_planner_test.cc
  package_test.cc
  prius_vis_test.cc
  pure_pursuit_controller_test.cc
  pure_pursuit_test.cc
  resources_test.cc
  simple_car_test.cc
  scene_system_test.cc
  simulation_run_stats_test.cc
  simulation_runner_test.cc
  time_conversion_test.cc
  traffic_pose_selector_test.cc
  trajectory_follower_test.cc
  trajectory_test.cc
)

delphyne_build_tests(${SOURCES})
