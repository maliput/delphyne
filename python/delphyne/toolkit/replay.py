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
import contextlib
import itertools
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


TOPIC_LOG_FILENAME = 'topic.db'
# Directory names in zip files have
# a trailing slash.
BUNDLE_DIRNAME = 'bundle/'


def is_log_file(path):
    """
    Checks whether the provided `path` points to a valid
    Delphyne log file or not.
    """
    if not path or not zipfile.is_zipfile(path):
        return False
    with zipfile.ZipFile(path) as archive:
        members = archive.namelist()
        return (TOPIC_LOG_FILENAME in members
                and BUNDLE_DIRNAME in members)


@contextlib.contextmanager
def open_log_file(path):
    """
    Extracts the Delphyne log file at `path` at a temporary
    location and returns, in order of appearance, ignition
    transport topic logs and Delphyne bundled package as a
    a tuple.

    Upon context exit, extracted content is dropped.
    """
    with zipfile.ZipFile(path) as archive:
        tempdir = tempfile.mkdtemp()
        try:
            archive.extractall(tempdir)
            yield (os.path.join(tempdir, TOPIC_LOG_FILENAME),
                   os.path.join(tempdir, BUNDLE_DIRNAME))
        finally:
            shutil.rmtree(tempdir)


def parse_arguments():
    """Argument passing and demo documentation."""
    parser = argparse.ArgumentParser(
        description=delphyne.cmdline.create_argparse_description(
            "Simulation replaying tool",
            "A tool to replay a simulation from a log file."
        ),
        epilog=delphyne.cmdline.create_argparse_epilog(),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log_file', nargs='?',
                        help='Path to simulation log file.')
    parser.add_argument('-l', '--latest-log-file', action='store_true',
                        help="Use latest simulation log file.")
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

    if not args.log_file and args.latest_log_file:
        args.log_file = next(
            itertools.ifilter(is_log_file, sorted(
                os.listdir('.'), key=os.path.getmtime, reverse=True
            )), None
        )

    if not is_log_file(args.log_file):
        print("No valid log file to replay.")
        quit()
    print("Replaying logs at {}.".format(args.log_file))

    with open_log_file(args.log_file) as (topic_log_path, bundle_path):
        launch_manager = delphyne.launcher.Launcher()
        try:
            replayer = "replayer"
            launch_manager.launch([replayer, topic_log_path])
            if not args.bare:
                launch_visualizer(
                    launch_manager, VISUALIZER_LAYOUT, bundle_path
                )
                launch_manager.wait(float("Inf"))
        except RuntimeError as error_msg:
            sys.stderr.write("ERROR: {}".format(error_msg))
        finally:
            # This is needed to avoid a possible deadlock.
            # See SimulatorRunner class description.
            time.sleep(0.5)
            launch_manager.kill()
