#
# TODO(daniel.stonier): convert this into a proper python setuptools package
#
##############################################################################
# Setup
##############################################################################

include(${PROJECT_SOURCE_DIR}/cmake/Python.cmake)

##############################################################################
# Bindings
##############################################################################

add_subdirectory(bindings)

##############################################################################
# Package
##############################################################################

install(
  DIRECTORY delphyne
  DESTINATION ${PYTHON_INSTALL_DIR}
)

##############################################################################
# Examples
##############################################################################

install(
  PROGRAMS
    examples/all_cars_in_dragway.py
    examples/keyboard_controlled_simulation.py
    examples/railcar_in_multilane.py
    examples/realtime_rate_changer.py
    examples/road_loading.py
    examples/simple_python_binding.py
    examples/start_simulation_paused.py
    examples/time_bounded_simulation.py
  DESTINATION
    bin
)
