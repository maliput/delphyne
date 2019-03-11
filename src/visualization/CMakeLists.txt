##############################################################################
# Ignition Visualization
##############################################################################

add_library(visualization
  box_car_vis.cc
  car_vis.cc
  car_vis_applicator.cc
  prius_vis.cc
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
    ignition-common2::ignition-common2
)

install(TARGETS visualization
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
