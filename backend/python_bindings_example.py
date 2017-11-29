#!/usr/bin/python2.7

import os
import select
import sys
import time
from launcher import Launcher
from simulation_runner_py import SimulatorRunner

"""This is a minimal example of starting an automotive simulation using a python
binding to the C++ SimulatorRunner class. Note that this is not a configurable
demo, it will just create a sample simulation with a prius car that can be
driven around. As we add more python bindings to the C++ classes we will start
doing more interesting scripts"""


def get_from_env_or_fail(var):
    value = os.environ.get(var)
    if value is None:
        print("%s is not in the environment, did you remember to source setup.bash?" % (var))
        sys.exit(1)

    # Since it is an environment variable, the very end may have a colon; strip it here
    if value[-1] == ':':
        value = value[:-1]

    return value

def main():
    launcher = Launcher()

    delphyne_ws_dir = get_from_env_or_fail('DELPHYNE_WS_DIR')
    lcm_ign_bridge = "duplex-ign-lcm-bridge"
    ign_visualizer = "visualizer"

    try:
        launcher.launch([lcm_ign_bridge, "1"])
        teleop_config = os.path.join(delphyne_ws_dir, "install", "share", "delphyne", "layoutWithTeleop.config")
        launcher.launch([ign_visualizer, teleop_config])

        # We need this to make sure the visualizer is up an running before we
        # send the initial message to load the robot. This will go away once
        # we have ported the automotive simulator to the backend.
        time.sleep(1.0)

        runner = SimulatorRunner();
        runner.start();

        launcher.wait(float("Inf"))

    finally:
        launcher.kill()

if __name__ == "__main__":
    main()
