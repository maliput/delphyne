#!/usr/bin/env python2.7
#
# Copyright 2017 Toyota Research Institute
#

"""
Thin wrapper around the python_bindigng.so lib generated in C++ that ensures
proper symbol loading.
"""

# There are a bunch of things we do here that makes pylint complain, so
# shutting it down for good.
# pylint: skip-file

from __future__ import print_function

import sys

RTLD_NOW = 0x00002
RTLD_GLOBAL = 0x00100

original_dl_open_flags = sys.getdlopenflags()

try:
    sys.setdlopenflags(RTLD_NOW | RTLD_GLOBAL)
    from python_bindings import *
finally:
    sys.setdlopenflags(original_dl_open_flags)
