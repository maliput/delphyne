##############################################################################
# Libraries for the agent plugin loader test
##############################################################################

################################## Macro #####################################

macro(delphyne_add_agent_test_library name)
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
endmacro()

################################# Agents #####################################

delphyne_add_agent_test_library(simple_agent)
delphyne_add_agent_test_library(auto_diff_agent)
delphyne_add_agent_test_library(symbolic_agent)

