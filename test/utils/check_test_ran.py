#!/usr/bin/env python3

"""
Writes a test failure out to test file if it doesn't exist.
"""

# Adapted from rosunit/check_test_ran.py

import os
import sys

NAME = "check_test_ran.py"


def usage():
    print("""Usage: \t%s test-file.xml """ % (NAME))
    print(sys.argv)
    sys.exit(getattr(os, 'EX_USAGE', 1))


def check_main():
    if len(sys.argv) < 2:
        usage()
    test_file = sys.argv[1]

    print("Checking for test results in %s" % test_file)

    if not os.path.exists(test_file) or os.path.getsize(test_file) == 0:
        if not os.path.exists(os.path.dirname(test_file)):
            os.makedirs(os.path.dirname(test_file))

        print("Cannot find results, writing failure results to", test_file)

        with open(test_file, 'w') as f:
            test_name = os.path.basename(test_file)
            d = {'test': test_name, 'test_file': test_file,
                 'test_no_xml': test_name.replace('.xml', '')}
            f.write("""<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="1" time="1" errors="0" name="%(test)s">
  <testcase name="test_ran" status="run" time="1" classname="%(test_no_xml)s">
    <failure message="Unable to find test results for %(test)s, test did not run.\nExpected results in %(test_file)s" type=""/>
  </testcase>
</testsuite>""" % d)  # noqa: E501


if __name__ == '__main__':
    check_main()
