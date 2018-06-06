##############################################################################
# Modules
##############################################################################

install(
  FILES
    __init__.py
    console.py
    launcher.py
    simulation_utils.py
  DESTINATION
    ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

##############################################################################
# Bindings
##############################################################################

####################
# Agent Bindings
####################

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

####################
# Maliput Bindings
####################

pybind11_add_module(maliput_bindings maliput.cc)
set_target_properties(maliput_bindings PROPERTIES OUTPUT_NAME maliput)

target_link_libraries(maliput_bindings
  PRIVATE
    maliput_utilities
    drake::drake
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(
  TARGETS maliput_bindings
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

####################
# Sim Bindings
####################

pybind11_add_module(simulation_bindings simulation.cc)
set_target_properties(simulation_bindings PROPERTIES OUTPUT_NAME simulation)

target_link_libraries(simulation_bindings
  PRIVATE
    simulation_runner
    automotive_simulator
    drake::drake
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(
  TARGETS simulation_bindings
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
