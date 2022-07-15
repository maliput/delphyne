##############################################################################
# Modules
##############################################################################

install(
  FILES
    __init__.py
    cmdline.py
    console.py
    utilities.py
    trees.py
  DESTINATION
    ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

install(
  DIRECTORY
    behaviours
    blackboard
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
    delphyne::agents
    delphyne::backend
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
    drake::drake
    maliput::api
    maliput::math
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(
  TARGETS maliput_bindings
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

####################
# Roads Bindings
####################

pybind11_add_module(roads_bindings roads.cc)
set_target_properties(roads_bindings PROPERTIES OUTPUT_NAME roads)

target_link_libraries(roads_bindings
  PRIVATE
    drake::drake
    delphyne::roads_utilities
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(
  TARGETS roads_bindings
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

####################
# Sim Bindings
####################

pybind11_add_module(simulation_bindings simulation.cc)
set_target_properties(simulation_bindings PROPERTIES OUTPUT_NAME simulation)

target_include_directories(
  simulation_bindings
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/python>
)

target_link_libraries(simulation_bindings
  PRIVATE
    drake::drake
    delphyne::backend
    pybind11::module
    ${PYTHON_LIBRARIES}
)

install(
  TARGETS simulation_bindings
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
