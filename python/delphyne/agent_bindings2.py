#!/usr/bin/env python2.7
#
# Copyright 2018 Toyota Research Institute
#

#############################################################################
# Documentation
#############################################################################

"""
Agent bindings.
"""

#############################################################################
# Bindings
#############################################################################

# There are a bunch of things we do here that makes pylint complain (variable
# name when defined outside main(), using from ... import * that is not at the
# top of the file, etc), so shutting it down for good.
# pylint: skip-file

import sys

RTLD_NOW = 0x00002
RTLD_GLOBAL = 0x00100

original_dl_open_flags = sys.getdlopenflags()

try:
    # Make sure that the symbols loaded from this library (that in turn
    # references Drake ones) are available to other dl_open'ed libraries (e.g.
    # the dynamically loaded agents).
    sys.setdlopenflags(RTLD_NOW | RTLD_GLOBAL)
    from agent_bindings import *
finally:
    sys.setdlopenflags(original_dl_open_flags)
