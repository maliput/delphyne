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
  # + tools/run_tests.sh
  DELPHYNE_BUILD_DIR=$DELPHYNE_SOURCE_DIR/build/delphyne
  DELPHYNE_INSTALL_DIR=$DELPHYNE_SOURCE_DIR/install
else
  # If `-jenkins` was NOT defined, it assumes the
  # following conventions about the workspace:
  # + build/delphyne
  # + src/delphyne/tools/run_tests.sh
  DELPHYNE_BUILD_DIR=$( dirname $( dirname $DELPHYNE_SOURCE_DIR) )/build/delphyne
  DELPHYNE_INSTALL_DIR=$( dirname $( dirname $DELPHYNE_SOURCE_DIR) )/install
fi

EXIT=0

export PYTHONPATH=$DELPHYNE_INSTALL_DIR/lib/python2.7/site-packages:$DELPHYNE_INSTALL_DIR/lib
export LD_LIBRARY_PATH=$DELPHYNE_BUILD_DIR/backend:$DELPHYNE_INSTALL_DIR/lib

pushd $DELPHYNE_BUILD_DIR

printf "\nRunning C++ tests:\n"
cmake $DELPHYNE_SOURCE_DIR -DCMAKE_INSTALL_PREFIX=$DELPHYNE_INSTALL_DIR
make -j$( getconf _NPROCESSORS_ONLN )
CTEST_OUTPUT_ON_FAILURE=1 make test || EXIT=$?
popd

# TODO(basicNew): Bring python tests back
# printf "\nRunning Python tests:\n"
# # TODO(apojomovsky): replace this by `python setup.py test` after #290 is set up.
# python -m unittest discover backend/test "*_test.py" || EXIT=$?

# Since C++ tests can exit with an exit code different than 1 on failure,
# we unify them so that we always get 1 on failure and 0 on success.
if [ "$EXIT" -ne 0 ]; then
  printf "\nSome tests failed.\n"
  EXIT=1
else
  printf "\nAll tests passed.\n"
fi

exit $EXIT
