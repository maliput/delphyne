#!/usr/bin/env python2.7
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

 * ``Regular``: black, red, green, yellow, blue, magenta, cyan, white
 * ``Bold``: bold, bold_black, bold_red, bold_green, bold_yellow,
             bold_blue, bold_magenta, bold_cyan, bold_white

These colour definitions can be used in the following way:
.. code-block:: python
   import delphyne.console as console
   print(console.cyan + "    Name" + console.reset + ": " + console.yellow +
         "Dude" + console.reset)

Note: 'colorama' is a drop-in replacement for this module (with actual
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

has_colours = console_has_colours()
""" Whether the loading program has access to colours or not."""

if has_colours:
    reset = "\x1b[0m"
    bold = "\x1b[%sm" % '1'
    black, red, green, yellow, blue, magenta, cyan, white = ["\x1b[%sm" % str(i) for i in range(30, 38)]
    bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white = ["\x1b[%sm" % ('1;' + str(i)) for i in range(30, 38)]
else:
    reset = ""
    bold = ""
    black, red, green, yellow, blue, magenta, cyan, white = ["" for i in range(30, 38)]
    bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white = ["" for i in range(30, 38)]

colours = [bold,
           black, red, green, yellow, blue, magenta, cyan, white,
           bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white
           ]
"""List of all available colours."""

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    for colour in colours:
        print(colour + "dude" + reset)
    print("some normal text")
    print(cyan + "    Name" + reset + ": " + yellow + "Dude" + reset)