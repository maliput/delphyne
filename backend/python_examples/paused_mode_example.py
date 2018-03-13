#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
This example show how to use the `SimulationRunner`'s (optional) third
constructor argument to start in pause mode.

```
$ cd <delphyne_ws>/install/bin
$ ./paused_mode_example.py
```

This command will spawn a visualizer instance and start the simulation
in paused mode.

 ## Unpausing the simulation

- At this moment, the only available way of achieving this is by making
use of a `WorldControl` service, publishing a message with the right
content into the `/world_control` channel:

```
$ cd <delphyne_ws>/install/bin
$ ./ign service --service /world_control --reqtype ignition.msgs.WorldControl \
--reptype ignition.msgs.Boolean --timeout 500 --req 'pause: false'
```

- An alternative way will be to use the `TimePanel` widget in the visualizer
(currently under development), which will allow to control the simulation with
`Play` / `Pause` / `Step` buttons from the GUI.

"""

from __future__ import print_function

import time

from launcher import Launcher
from python_bindings import SimulatorRunner
from delphyne_utils import (
    build_simple_car_simulator,
    launch_visualizer
)


def main():
    """Spawns a simulator_runner in paused mode"""
    launcher = Launcher()

    simulator = build_simple_car_simulator([(-5.0, -7.0), (1.0, -4.0)])

    # Use the optional third argument to instantiate a
    # simulator runner in paused mode
    start_paused = True
    runner = SimulatorRunner(simulator, 0.001, start_paused)

    try:
        launch_visualizer(launcher, "layoutWithTeleop.config")
        runner.Start()
        launcher.wait(float("Inf"))
    finally:
        runner.Stop()
        # This is needed to avoid a possible deadlock. See SimulatorRunner
        # class description.
        time.sleep(0.5)
        launcher.kill()


if __name__ == "__main__":
    main()
