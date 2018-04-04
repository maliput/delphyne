#!/usr/bin/env bash

set -e

SCRIPT_PATH=$( realpath ${BASH_SOURCE[0]} )
SCRIPT_DIR=$( dirname $SCRIPT_PATH )
DELPHYNE_SOURCE_DIR=$( dirname $SCRIPT_DIR )

if [ "$1" == "-jenkins" ];
then
  # If `-jenkins` was defined, it assumes the
  # following conventions about the workspace:
  # + build/delphyne
  # + test/run_tests.sh
  DELPHYNE_BUILD_DIR=$DELPHYNE_SOURCE_DIR/build/delphyne
  DELPHYNE_INSTALL_DIR=$DELPHYNE_SOURCE_DIR/install
else
  # If `-jenkins` was NOT defined, it assumes the
  # following conventions about the workspace:
  # + build/delphyne
  # + src/delphyne/test/run_tests.sh
  DELPHYNE_BUILD_DIR=$( dirname $( dirname $DELPHYNE_SOURCE_DIR) )/build/delphyne
  DELPHYNE_INSTALL_DIR=$( dirname $( dirname $DELPHYNE_SOURCE_DIR) )/install
fi

pushd $DELPHYNE_BUILD_DIR

EXIT=0

#################################### BUILD ###################################

printf "\nBuilding and Installing:\n"
cmake $DELPHYNE_SOURCE_DIR -DCMAKE_INSTALL_PREFIX=$DELPHYNE_INSTALL_DIR
make -j$( getconf _NPROCESSORS_ONLN )

##################################### CPP ####################################

printf "\nRunning C++ tests:\n"
CTEST_OUTPUT_ON_FAILURE=1 make test || EXIT=$?
popd

# Since C++ tests can exit with an exit code different than 1 on failure,
# we unify them so that we always get 1 on failure and 0 on success.
if [ "$EXIT" -ne 0 ]; then
  printf "\nSome c++ tests failed.\n"
  exit 1
else
  printf "\nAll c++ tests passed.\n"
fi

################################### PYTHON ###################################

export PYTHONPATH=$DELPHYNE_INSTALL_DIR/lib/python2.7/site-packages:$DELPHYNE_INSTALL_DIR/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DELPHYNE_INSTALL_DIR/lib

printf "\nRunning Python tests:\n"
# TODO(apojomovsky): replace this by `python setup.py test` after #290 is set up.
python -m unittest discover ${SCRIPT_DIR}/regression/python "*_test.py" || EXIT=$?

# Since C++ tests can exit with an exit code different than 1 on failure,
# we unify them so that we always get 1 on failure and 0 on success.
if [ "$EXIT" -ne 0 ]; then
  printf "\nSome python tests failed.\n"
  exit 1
else
  printf "\nAll python tests passed.\n"
fi

exit 0

