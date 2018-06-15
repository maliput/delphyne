#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#
##############################################################################
# Documentation
##############################################################################

"""Utility methods for the demos."""

##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import argparse

import delphyne.console as console

##############################################################################
# Argparsing
##############################################################################


def create_argument_parser(title, content, default_duration=-1.0):
    """
    Create an argument parser for use with the demos and
    populate it with some common arguments.
    Args:
        title: short, descriptive title for the demo
        content: longer, detailed description of the demo
        default_duration: default length of the simulation (s)
    Returns:
        argparse.ArgParser: the initialised argument parser
    """

    def check_positive_float_or_zero(value):
        """Check that the passed argument is a positive float value"""
        float_value = float(value)
        if float_value < 0.0:
            raise argparse.ArgumentTypeError("%s is not a positive float value"
                                             % value)
        return float_value

    parser = argparse.ArgumentParser(
        description=create_argparse_description(title, content),
        epilog=create_argparse_epilog(),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-d", "--duration", default=default_duration,
                        help="Simulation length (sec)(endless if -ve)"
                        "(default: {0}s)".format(default_duration))
    parser.add_argument("-r", "--realtime_rate", default=1.0,
                        type=check_positive_float_or_zero,
                        help="Ratio of sim vs real time (default: 1.0)")
    parser.add_argument('-p', '--paused',
                        action='store_true',
                        help='Start the simulation paused (default: False)')
    return parser


def create_argparse_description(title, content):
    """
    Format an argparse description in a nice way with a banner + title and
    content beneath.
    Args:
        title:
        content:
    """
    if console.HAS_COLOURS:
        banner_line = console.GREEN + "*" * 70 + "\n" + console.RESET
        desc = "\n"
        desc += banner_line
        desc += console.BOLD_WHITE + title.center(70) + "\n" + console.RESET
        desc += banner_line
        desc += content
        desc += "\n"
        desc += banner_line
    else:
        desc = content
    return desc


# pylint: disable=inconsistent-return-statements
def create_argparse_epilog():
    """
    Create a humourous anecdote for argparse's epilog.
    """
    if console.HAS_COLOURS:
        msg = "And his noodly appendage reached forth to "\
              "tickle the blessed...\n"
        return console.CYAN + msg + console.RESET
