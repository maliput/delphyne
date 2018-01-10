#!/usr/bin/env python2.7

"""This example is aimed to show the use of the SimulationRunner
making use of the a third (optional) argument in the constructor, which
enables the simulator to start in paused mode.
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
from select import select

import argparse
import atexit
import os
import sys
import termios
import time

from launcher import Launcher
from pydrake.common import AddResourceSearchPath
from simulation_runner_py import (
    AutomotiveSimulator,
    SimpleCarState,
    SimulatorRunner
)
from utils import (
    get_from_env_or_fail,
    build_automotive_simulator
)


"""
In order to run this example, you must first move into the
<delphyne_ws>/install/bin directory and then call:

./paused_mode_example.py --paused

This command will spawn a visualizer instance and start a simulation in paused mode.

In order to unpause the simulation, there are to different approaches available:

- The first way involves making use of a WorldControl service, by publishing a
WorldControl message into the /world_control channel with the right command.
In order to unpause the simulation with a service request, you can execute the
following command from the <delphyne_ws>/install/bin directory:

./ign service --service /world_control --reqtype ignition.msgs.WorldControl \
--reptype ignition.msgs.Boolean --timeout 500 --req 'pause: false'

- The second way is still unavailable, but worth mentioning here since it's aimed
to be the default way of controlling the simulation once it's ready.
It involves making use of the TimePanel widget in the visualizer (currently under
development), which should enable the user to control the simulation with buttons
like Play, Pause, Step, all from the Visualizer's GUI.
"""
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--paused", action='store_true',
                        dest='start_paused',
                        default=False, help="Start simulator in paused mode")
    args = parser.parse_args()

    """Spawn an automotive simulator"""
    launcher = Launcher()

    delphyne_ws_dir = get_from_env_or_fail('DELPHYNE_WS_DIR')
    lcm_ign_bridge = "duplex-ign-lcm-bridge"
    ign_visualizer = "visualizer"

    drake_install_path = get_from_env_or_fail('DRAKE_INSTALL_PATH')
    AddResourceSearchPath(os.path.join(drake_install_path, "share", "drake"))

    simulator = build_automotive_simulator()

    # Use the optional third argument to instantiate a
    # simulator runner in paused mode
    runner = SimulatorRunner(simulator, 0.001, args.start_paused)

    try:
        launcher.launch([lcm_ign_bridge, "1"])
        teleop_config = os.path.join(delphyne_ws_dir,
                                     "install",
                                     "share",
                                     "delphyne",
                                     "layoutWithTeleop.config")
        launcher.launch([ign_visualizer, teleop_config])

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
