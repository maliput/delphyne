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

ament_target_dependencies(visualization
"drake"
"maliput"
"dragway"
)

target_link_libraries(visualization
    public_headers
    drake::drake
    maliput::api
    maliput::utilities
    dragway::dragway
    ignition-common3::ignition-common3
)

install(TARGETS visualization
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
