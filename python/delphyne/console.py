#!/usr/bin/env python3
#
# Copyright 2017 Toyota Research Institute
#
##############################################################################
# Description
##############################################################################

"""
Simple colour definitions for consoles.
----

**Colour Definitions**

The current list of colour definitions include:

 * ``Regular``: BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE
 * ``Bold``: BOLD, BOLD_BLACK, BOLD_RED, BOLD_GREEN, BOLD_YELLOW,
             BOLD_BLUE, BOLD_MAGENTA, BOLD_CYAN, BOLD_WHITE

These colour definitions can be used in the following way:
.. code-block:: python
   import delphyne.console as console
   print(console.CYAN + "    Name" + console.RESET + ": " + console.YELLOW +
         "Dude" + console.RESET)

..note:: 'colorama' is a drop-in replacement for this module (with actual
cross-platform support) with the only caveat being that it does not have
the ability to determine whether colour is available.
"""

##############################################################################
# Imports
##############################################################################

import os
import sys

##############################################################################
# Methods
##############################################################################


def console_has_colours():
    """
    Detects if the console (stdout) has colourising capability.
    """
    if os.environ.get("PY_TREES_DISABLE_COLORS"):
        return False
    # From django.core.management.color.supports_color
    #   https://github.com/django/django/blob/master/django/core/management/color.py
    plat = sys.platform
    supported_platform = plat != 'Pocket PC' and (plat != 'win32' or
                                                  'ANSICON' in os.environ)
    # isatty is not always implemented, #6223.
    is_a_tty = hasattr(sys.stdout, 'isatty') and sys.stdout.isatty()
    if not supported_platform or not is_a_tty:
        return False
    return True


HAS_COLOURS = console_has_colours()
""" Whether the loading program has access to colours or not."""


if HAS_COLOURS:
    RESET = "\x1b[0m"
    BOLD = "\x1b[%sm" % '1'
    BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = [
        "\x1b[%sm" % str(i) for i in range(30, 38)]
    [BOLD_BLACK, BOLD_RED, BOLD_GREEN, BOLD_YELLOW, BOLD_BLUE,
     BOLD_MAGENTA, BOLD_CYAN, BOLD_WHITE] = [
         "\x1b[%sm" % ('1;' + str(i)) for i in range(30, 38)]
else:
    RESET = ""
    BOLD = ""
    BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = [
        "" for i in range(30, 38)]
    [BOLD_BLACK, BOLD_RED, BOLD_GREEN, BOLD_YELLOW, BOLD_BLUE,
     BOLD_MAGENTA, BOLD_CYAN, BOLD_WHITE] = [
         "" for i in range(30, 38)]

COLOURS = [BOLD,
           BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE,
           BOLD_BLACK, BOLD_RED, BOLD_GREEN, BOLD_YELLOW, BOLD_BLUE,
           BOLD_MAGENTA, BOLD_CYAN, BOLD_WHITE]
"""List of all available colours."""

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    for colour in COLOURS:
        print(colour + "dude" + RESET)
    print("some normal text")
    print(CYAN + "    Name" + RESET + ": " + YELLOW + "Dude" + RESET)
