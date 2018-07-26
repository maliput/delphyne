#
# Copyright 2018 Toyota Research Institute
#
##############################################################################
# Documentation
##############################################################################

"""Utility functions for command line tools."""

##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import delphyne.console as console

##############################################################################
# Argument parsing
##############################################################################


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


def create_argparse_epilog():
    """
    Create a humourous anecdote for argparse's epilog.
    """
    msg = "And his noodly appendage reached forth to "\
          "tickle the blessed...\n"
    if console.HAS_COLOURS:
        return console.CYAN + msg + console.RESET
    return None
