##############################################################################
# Macros
##############################################################################

macro(delphyne_add_agent_library name)
  add_library(${name} ${name}.cc)
  string(REPLACE "_" "-" LIBRARY_NAME ${name})
  set_target_properties(${name} PROPERTIES OUTPUT_NAME ${LIBRARY_NAME})
  target_link_libraries(
    ${name}
    PRIVATE
      drake_systems
      drake::drake
      Eigen3::Eigen
      ignition-common2::ignition-common2
      stx::stx
  )
  install(TARGETS ${name}
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/delphyne/agents)
endmacro()

##############################################################################
# Agents
##############################################################################

delphyne_add_agent_library(rail_car)
delphyne_add_agent_library(mobil_car)

##############################################################################
# Agents
##############################################################################

add_library(agents
  simple_car.cc
  trajectory_agent.cc
)
set_target_properties(agents PROPERTIES OUTPUT_NAME ${PROJECT_NAME}-agents)

target_link_libraries(
  agents
  PUBLIC
    public_headers
    drake_systems
    drake::drake
    Eigen3::Eigen
)
install(TARGETS agents
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
