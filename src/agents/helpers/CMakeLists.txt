# ----------------------------------------
# Agent helpers library

add_library(agent_helpers frame_pose_aggregator.cc)

set_target_properties(agent_helpers PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-agent-helpers)
target_include_directories(
  agent_helpers
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)
target_link_libraries(
  agent_helpers
  PUBLIC
    public_headers
    ${drake_LIBRARIES}
)

install(TARGETS agent_helpers
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
