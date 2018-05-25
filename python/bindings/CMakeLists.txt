##############################################################################
# Private Header Access
##############################################################################

include_directories(${PROJECT_SOURCE_DIR}/src)

##############################################################################
# Bindings - Agents
##############################################################################

pybind11_add_module(agent_bindings agents.cc)

set_target_properties(agent_bindings PROPERTIES OUTPUT_NAME agents)

target_link_libraries(agent_bindings
  PRIVATE
    agents
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(
  TARGETS agent_bindings
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
