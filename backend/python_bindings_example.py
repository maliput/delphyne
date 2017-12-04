#!/usr/bin/env python
#
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

import os
import select
import sys
import time
from launcher import Launcher
from simulation_runner_py import SimulatorRunner

"""This is a minimal example of starting an automotive simulation using a
python binding to the C++ SimulatorRunner class. Note that this is not a
configurable demo, it will just create a sample simulation with a prius car
that can be driven around. As we add more python bindings to the C++ classes we
will start doing more interesting scripts"""


def get_from_env_or_fail(var):
    value = os.environ.get(var)
    if value is None:
        print("%s is not in the environment,"
              "did you remember to source setup.bash?" % (var))
        sys.exit(1)

    # Since it is an environment variable, the very end may have a colon;
    # strip it here
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
        teleop_config = os.path.join(delphyne_ws_dir,
                                     "install",
                                     "share",
                                     "delphyne",
                                     "layoutWithTeleop.config")
        launcher.launch([ign_visualizer, teleop_config])

        # TODO(basicNew) We need this to make sure the visualizer is up and
        # running before we send the initial message to load the robot.
        # This will go away once we have ported the automotive simulator to
        # the backend.
        time.sleep(1.0)

        runner = SimulatorRunner()
        runner.start()

        launcher.wait(float("Inf"))

    finally:
        launcher.kill()

if __name__ == "__main__":
    main()
