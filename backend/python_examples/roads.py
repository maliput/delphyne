#!/usr/bin/env python2.7

"""
This example shows how to run a simulation that includes a road. For the
time being two road examples are supported: dragway and onramp. To launch this
demo with the default values do:

```
$ roads.py
```

Or explicitly pass the desired road:

```
$ roads.py --road='dragway'
```

```
$ roads.py --road='dragway'
```

"""

#
# Copyright 2017 Toyota Research Institute
#

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
    """Simple demo that shows how to add a road to a simulation. Reads the
    road to be added from the command line args"""

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
