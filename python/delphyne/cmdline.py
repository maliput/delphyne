# BSD 3-Clause License
#
# Copyright (c) 2022, Woven Planet. All rights reserved.
# Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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
# Documentation
##############################################################################

"""Utility functions for command line tools."""

##############################################################################
# Imports
##############################################################################

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
