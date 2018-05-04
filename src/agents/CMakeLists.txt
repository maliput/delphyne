
################################## Macro #####################################

macro(delphyne_add_agent_library name)
  add_library(${name} ${name}.cc)
  string(REPLACE "_" "-" LIBRARY_NAME ${name})
  set_target_properties(${name} PROPERTIES OUTPUT_NAME ${LIBRARY_NAME})
  target_link_libraries(
    ${name}
    PRIVATE
      drake::drake
      Eigen3::Eigen
      ignition-common1::ignition-common1
      stx::stx
  )
  install(TARGETS ${name}
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/delphyne/agents)
endmacro()

################################## Agents ####################################

delphyne_add_agent_library(rail_car)
delphyne_add_agent_library(mobil_car)
delphyne_add_agent_library(simple_car)
delphyne_add_agent_library(trajectory_car)

