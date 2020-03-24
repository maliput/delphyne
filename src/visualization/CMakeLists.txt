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
"maliput_dragway"
)

target_link_libraries(visualization
    drake::drake
    ignition-common3::ignition-common3
    maliput::api
    maliput::utilities
    maliput_dragway::maliput_dragway
    public_headers
)

install(TARGETS visualization
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
