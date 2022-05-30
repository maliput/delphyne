#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright (c) 2022, Woven Planet. All rights reserved.
# Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
