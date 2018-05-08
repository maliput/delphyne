#
# TODO(daniel.stonier): convert this into a proper python setuptools package
#
##############################################################################
# Python Module
##############################################################################

install(
  DIRECTORY delphyne
  DESTINATION lib/python2.7/site-packages
)

##############################################################################
# Python Examples
##############################################################################

install(
  PROGRAMS
    examples/all_cars_in_dragway.py
    examples/demo_launcher.py
    examples/keyboard_controlled_simulation.py
    examples/loadable_agent_simulation.py
    examples/railcar_in_multilane.py
    examples/realtime_rate_changer.py
    examples/road_loading.py
    examples/simple_python_binding.py
    examples/start_simulation_paused.py
    examples/time_bounded_simulation.py
  DESTINATION
    bin
)
