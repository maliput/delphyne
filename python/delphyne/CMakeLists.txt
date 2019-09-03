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

pybind11_add_module(agent_bindings MODULE agents.cc)
set_target_properties(agent_bindings PROPERTIES OUTPUT_NAME agents)
if(PYTHON_IS_DEBUG)
  target_compile_definitions(agent_bindings PRIVATE Py_DEBUG)
endif()


target_link_libraries(agent_bindings
  PRIVATE
    agents
    backend
)

install(
  TARGETS agent_bindings
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

####################
# Maliput Bindings
####################

pybind11_add_module(roads_bindings MODULE roads.cc)
set_target_properties(roads_bindings PROPERTIES OUTPUT_NAME roads)
if(PYTHON_IS_DEBUG)
  target_compile_definitions(roads_bindings PRIVATE Py_DEBUG)
endif()

target_link_libraries(roads_bindings
  PRIVATE
    roads_utilities
    drake::drake
)

install(
  TARGETS roads_bindings
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

####################
# Sim Bindings
####################

pybind11_add_module(simulation_bindings MODULE simulation.cc)
set_target_properties(simulation_bindings PROPERTIES OUTPUT_NAME simulation)
if(PYTHON_IS_DEBUG)
  target_compile_definitions(simulation_bindings PRIVATE Py_DEBUG)
endif()

target_include_directories(
  simulation_bindings
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/python>
)

target_link_libraries(simulation_bindings
  PRIVATE
    backend
    drake::drake
)

install(
  TARGETS simulation_bindings
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
