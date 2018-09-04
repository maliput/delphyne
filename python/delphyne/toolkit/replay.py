#!/usr/bin/env python2.7
#
# Copyright 2018 Toyota Research Institute
#

"""
The replay tool.
"""

##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import argparse
import os
import shutil
import sys
import tempfile
import time
import zipfile

import delphyne.cmdline
import delphyne.launcher

from delphyne.utilities import (
    launch_visualizer
)


##############################################################################
# Supporting Classes & Methods
##############################################################################


def parse_arguments():
    """Argument passing and demo documentation."""
    parser = argparse.ArgumentParser(
        description=delphyne.cmdline.create_argparse_description(
            "Simulation replaying tool",
            "A tool to replay a simulation from a log file."
        ),
        epilog=delphyne.cmdline.create_argparse_epilog(),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log_file', help='Path to simulation log file.')
    parser.add_argument(
        '-b', '--bare', action='store_true', default=False,
        help="If true, replay without visualization (default: False)."
    )
    return parser.parse_args()


##############################################################################
# Main
##############################################################################


VISUALIZER_LAYOUT = "layout_for_playback.config"


def main():
    """Keeping pylint entertained."""
    args = parse_arguments()

    with zipfile.ZipFile(args.log_file) as archive:
        tempdir = tempfile.mkdtemp()
        try:
            archive.extractall(tempdir)
            launch_manager = delphyne.launcher.Launcher()
            try:
                replayer = "replayer"
                topic_log_path = os.path.join(tempdir, "topic.db")
                launch_manager.launch([replayer, topic_log_path])
                if not args.bare:
                    launch_visualizer(launch_manager, VISUALIZER_LAYOUT,
                                      os.path.join(tempdir, "bundle"))
                launch_manager.wait(float("Inf"))
            except RuntimeError as error_msg:
                sys.stderr.write("ERROR: {}".format(error_msg))
            finally:
                # This is needed to avoid a possible deadlock.
                # See SimulatorRunner class description.
                time.sleep(0.5)
                launch_manager.kill()
        finally:
            shutil.rmtree(tempdir)
