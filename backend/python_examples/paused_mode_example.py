#!/usr/bin/env python2.7

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

# Copyright 2017 Open Source Robotics Foundation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import sys
import time

from launcher import Launcher
from simulation_runner_py import SimulatorRunner
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
