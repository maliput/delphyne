#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
This example show how to use the `SimulationRunner`'s (optional) third
constructor argument to start in pause mode.

```
$ cd <delphyne_ws>/install/bin
$ ./start_simulation_paused.py
```

This command will spawn a visualizer instance and start the simulation in
paused mode.

 ## Unpausing the simulation

- The most user-friendly way of unpausing the simulation is by using the
`TimePanel` widget in the visualizer. This panel allows to control the
simulation with `Play` / `Pause` / `Step` buttons from the GUI.

- Through the command line, by using the `WorldControl` service, publishing a
message with the right content into the `/world_control` channel:

```
$ cd <delphyne_ws>/install/bin
$ ./ign service --service /world_control --reqtype ignition.msgs.WorldControl \
--reptype ignition.msgs.Boolean --timeout 500 --req 'pause: false'
```

"""

from __future__ import print_function

from delphyne import SimulatorRunner
from simulation_utils import (
    build_simple_car_simulator,
    launch_interactive_simulation
)

SIMULATION_TIME_STEP_SECS = 0.001


def main():
    """Spawns a simulator_runner in paused mode"""

    simulator = build_simple_car_simulator([(-5.0, -7.0), (1.0, -4.0)])

    # Use the optional third argument to instantiate a
    # simulator runner in paused mode
    start_paused = True

    runner = SimulatorRunner(simulator,
                             SIMULATION_TIME_STEP_SECS,
                             start_paused)

    with launch_interactive_simulation(runner):
        runner.Start()


if __name__ == "__main__":
    main()
