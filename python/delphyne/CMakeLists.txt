##############################################################################
# Modules
##############################################################################

install(
  FILES
    __init__.py
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
# Other Bindings
####################
# TODO(daniel.stonier) break it down, maliput/simulator etc

pybind11_add_module(other_bindings bindings.cc)
set_target_properties(other_bindings PROPERTIES OUTPUT_NAME bindings)

target_link_libraries(other_bindings
  PRIVATE
    simulation_runner
    automotive_simulator
    drake::drake
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(
  TARGETS other_bindings
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
