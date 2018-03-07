#!/usr/bin/env bash

set -e

# Assumes some conventions about our workspace, namely:
# + build/delphyne
# + src/delphyne/tools/run_coverage.sh

SCRIPT_PATH=$( realpath ${BASH_SOURCE[0]} )
SCRIPT_DIR=$( dirname $SCRIPT_PATH )
DELPHYNE_SOURCE_DIR=$( dirname $SCRIPT_DIR )
DELPHYNE_BUILD_DIR=$( dirname $( dirname $DELPHYNE_SOURCE_DIR) )/build/delphyne

cd $DELPHYNE_BUILD_DIR
cmake $DELPHYNE_SOURCE_DIR -DCMAKE_INSTALL_PREFIX=../../install -DCMAKE_BUILD_TYPE=coverage
make -j$( getconf _NPROCESSORS_ONLN )
make test
make coverage
sensible-browser coverage/index.html
