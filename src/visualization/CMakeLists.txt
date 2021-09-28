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

add_library(delphyne::visualization ALIAS visualization)

set_target_properties(visualization
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_visualization
)

target_include_directories(
  visualization
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

target_link_libraries(visualization
  drake::drake
  delphyne::public_headers
  ignition-common4::ignition-common4
  maliput::api
  maliput::utilities
  maliput_dragway::maliput_dragway
)

install(
  TARGETS visualization
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
