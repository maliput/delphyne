#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""Launches drake's automotive_demo and delphyne's visualizer.
"""

from __future__ import print_function

import argparse
import os
from delphyne.launcher import Launcher
from delphyne.simulation_utils import get_from_env_or_fail


def main():
    """Launches drake's automotive_demo and delphyne's visualizer.
    Terminates all the processes if one of them is killed.

    Important: Many parts of this file are commented out as we are currently
    migrating the demos and not all features are supported. We are leaving
    these commented-out lines as a reference for the future.
    """
    launcher = Launcher()
    parser = argparse.ArgumentParser()
    demo_arguments = {
        "simple": ["--num_simple_car=1"]
    }

    # Optional arguments
    parser.add_argument("--demo", default="simple", dest="demo_name",
                        action="store", choices=demo_arguments.keys(),
                        help="the specific demo to launch")

    args = parser.parse_args()

    delphyne_resource_root = get_from_env_or_fail('DELPHYNE_RESOURCE_ROOT')

    # Delphyne binaries; these are found through the standard PATH, so
    # they are relative
    ign_visualizer = "visualizer"
    automotive_demo = "automotive-demo"

    try:
        if args.demo_name == "simple":
            # Load custom layout with two TeleopWidgets
            layout_key = "--layout="
            teleop_config = layout_key + os.path.join(
                delphyne_resource_root,
                "layoutWithTeleop.config")
            launcher.launch([ign_visualizer, teleop_config])
        else:
            launcher.launch([ign_visualizer])

        launcher.launch([automotive_demo])
        launcher.wait(float("Inf"))

    finally:
        launcher.kill()


if __name__ == "__main__":
    main()
