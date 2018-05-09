#!/bin/bash

set -e

# This script assumes some conventions about our workspace, namely:
# + build/delphyne
# + src/delphyne/tools/generate_python_docs.sh

SCRIPT_PATH=$( realpath ${BASH_SOURCE[0]} )
SCRIPT_DIR=$( dirname $SCRIPT_PATH )
DELPHYNE_SOURCE_DIR=$( dirname $SCRIPT_DIR )

# Enable pydocmd to find the examples
PYTHONPATH=${DELPHYNE_SOURCE_DIR}/python/examples:${PYTHONPATH}

# Sets the path to the generated readme.
PATH_TO_README=${DELPHYNE_SOURCE_DIR}/python/examples/README.md

# Remove previous version of README.md .
rm -f $PATH_TO_README && touch $PATH_TO_README

cat ${DELPHYNE_SOURCE_DIR}/tools/python_examples_md_template.in > $PATH_TO_README

cd ${DELPHYNE_SOURCE_DIR}/python/examples

# Converts a python module docstring into markdown and appends it to the README.
find . -iname '*.py' -printf '%f\n' | while read file ; do
  module_name=$(echo $file | cut -f 1 -d '.')
  echo "Processing $module_name"
  pydocmd simple $module_name >> $PATH_TO_README
done
