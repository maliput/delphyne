#!/usr/bin/env bash

set -e

cd $DELPHYNE_WS_DIR/build/delphyne
cmake ../../src/delphyne -DCMAKE_INSTALL_PREFIX=../../install -DCMAKE_BUILD_TYPE=coverage
make -j$( getconf _NPROCESSORS_ONLN )
make test
make coverage
sensible-browser coverage/index.html
