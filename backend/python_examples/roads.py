#!/usr/bin/env python2.7

"""
This example shows how to run a simulation for a fixed period of time (15
seconds by default). As an added bonus, the user can also specify the real-time
simulation rate (which by default is 1.0).

The following command depicts how to run a simulation for 30 sim seconds using
a real-time rate of 2x (so it should run in approximately 15 wall clock
seconds):

```
$ time_bounded_simulation.py --realtime_rate=2.0 --duration=30.0
```
"""

from __future__ import print_function

import argparse
import sys
import time

from launcher import Launcher
from python_bindings import (
    SimulatorRunner,
    RoadBuilder
)
from delphyne_utils import (
    build_simple_car_simulator,
    launch_visualizer
)


def main():

    available_roads = ["dragway", "onramp"]

    parser = argparse.ArgumentParser(
        prog="roads",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-r", "--road",
                    default=available_roads[0],
                    const=available_roads[0],
                    nargs='?',
                    choices=available_roads,
                    help='The road to display')

    args = parser.parse_args()

    road = args.road

    launcher = Launcher()

    simulator = build_simple_car_simulator()

    builder = RoadBuilder(simulator)

    if road == "dragway":
        builder.AddDragway("Demo dragway", 3, 100.0, 3.7, 1.0, 5.0)
    elif road == "onramp":
        builder.AddOnramp()
    else:
        raise RuntimeError("Option {} not recognized".format(road))

    runner = SimulatorRunner(simulator, 0.001)

    try:
        launch_visualizer(launcher, "layoutWithTeleop.config")

        runner.Start()

        launcher.wait(float("Inf"))

    except RuntimeError, error_msg:
        sys.stderr.write('ERROR: {}'.format(error_msg))
        sys.exit(1)

    finally:
        print("Simulation ended")
        # This is needed to avoid a possible deadlock. See SimulatorRunner
        # class description.
        time.sleep(0.5)
        launcher.kill()


if __name__ == "__main__":
    main()
