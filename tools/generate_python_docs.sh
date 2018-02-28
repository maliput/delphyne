#!/bin/bash

set -e

# Assumes some conventions about our workspace, namely:
# + build/delphyne
# + src/delphyne/tools/run_tests.sh

SCRIPT_PATH=$( realpath ${BASH_SOURCE[0]} )
SCRIPT_DIR=$( dirname $SCRIPT_PATH )
DELPHYNE_SOURCE_DIR=$( dirname $SCRIPT_DIR )

# Sets the path to the generated readme.
PATH_TO_README=${DELPHYNE_SOURCE_DIR}/backend/python_examples/README.md

# Remove previous version of README.md .
rm -f $PATH_TO_README && touch $PATH_TO_README

cat tools/python_examples_md_template.in > $PATH_TO_README

cd backend/python_examples

# Converts a python module docstring into markdown and appends it to the README.
find . -iname '*.py' -printf '%f\n' | while read file ; do
  module_name=$(echo $file | cut -f 1 -d '.')
  echo "Processing $module_name"
  pydocmd simple $module_name >> $PATH_TO_README
done
