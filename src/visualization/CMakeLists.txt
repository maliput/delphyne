##############################################################################
# Ignition Visualization
##############################################################################

add_library(visualization
  simple_prius_vis.cc
)
set_target_properties(visualization PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-visualization)
target_include_directories(
  visualization
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)
target_link_libraries(visualization
  PUBLIC
    public_headers
    drake::drake
)

install(TARGETS visualization
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
