#!/usr/bin/env python2.7

"""A group of common functions useful for the delphyne project"""

import os
import sys


def get_from_env_or_fail(var):
    """Retrieves an env variable for a given name, fails if not found."""
    value = os.environ.get(var)
    if value is None:
        print("%s is not in the environment, "
              "did you remember to source setup.bash?" % (var))
        sys.exit(1)

    # Since it is an environment variable, the very end may have a colon;
    # strip it here
    return value.rstrip(':')
