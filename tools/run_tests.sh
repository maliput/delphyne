#!/usr/bin/env bash

set -e

cd $DELPHYNE_WS_DIR/build/delphyne

printf "\nRunning C++ tests:\n"
cmake ../../src/delphyne -DCMAKE_INSTALL_PREFIX=../../install
make -j$( getconf _NPROCESSORS_ONLN )
make test

printf "\nRunning Python tests:\n"
python -m unittest discover backend "*_test.py"
