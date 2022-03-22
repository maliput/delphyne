##############################################################################
# Agents
##############################################################################

add_library(agents
  mobil_car.cc
  rail_car.cc
  rule_rail_car.cc
  simple_car.cc
  trajectory_agent.cc
  unicycle_car.cc
)

add_library(delphyne::agents ALIAS agents)

set_target_properties(agents
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_agents
)

target_include_directories(
  agents
  PUBLIC  # so the bindings can find these headers
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

target_link_libraries(
  agents
    drake::drake
    delphyne::drake_systems
    delphyne::mi6
    delphyne::public_headers
    delphyne::roads_utilities
    delphyne::translations
    maliput::api
    Eigen3::Eigen
)

install(
  TARGETS agents
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
