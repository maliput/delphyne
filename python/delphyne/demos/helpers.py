#!/usr/bin/env python3
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

import argparse

import delphyne.cmdline as cmdline

##############################################################################
# Argument parsing
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
        description=cmdline.create_argparse_description(title, content),
        epilog=cmdline.create_argparse_epilog(),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-d", "--duration", type=float,
                        default=default_duration,
                        help="Simulation length (sec)(endless if -ve)"
                        "(default: {0}s)".format(default_duration))
    parser.add_argument("-r", "--realtime_rate", default=1.0,
                        type=check_positive_float_or_zero,
                        help="Ratio of sim vs real time (default: 1.0)")
    parser.add_argument('-p', '--paused',
                        action='store_true',
                        help='Start the simulation paused (default: False)')
    parser.add_argument('-l', '--log',
                        action='store_true',
                        help='Log simulation data (default: False)')
    parser.add_argument('-f', '--logfile_name', default="",
                        action='store', type=str,
                        help='Custom logfile name (default: empty string)')
    parser.add_argument('-b', '--bare', action='store_true',
                        default=False, help=('Run simulation with no '
                                             'visualizer (default: False)'))
    return parser
