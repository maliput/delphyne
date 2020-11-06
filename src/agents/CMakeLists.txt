##############################################################################
# Agents
##############################################################################

add_library(agents
  mobil_car.cc
  rail_car.cc
  simple_car.cc
  trajectory_agent.cc
  unicycle_car.cc
)
set_target_properties(agents PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-agents)

target_include_directories(
  agents
  PUBLIC  # so the bindings can find these headers
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

ament_target_dependencies(agents
  "drake"
  "maliput"
)

target_link_libraries(
  agents
    mi6
    public_headers
    roads_utilities
    translations
    drake_systems
    drake::drake
    maliput::api
    Eigen3::Eigen
    translations
)
install(TARGETS agents
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
