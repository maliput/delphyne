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

CPP_EXIT=0
PYTHON_EXIT=0

#################################### BUILD ###################################

# Skip running cmake, CI has it's own rules, and we don't wan to override
# any rules the user specified in building it themselves.

if [ ! -d "${DELPHYNE_BUILD_DIR}" ]; then
  printf "\nBuild directory does not exist, make sure you have configured the build with cmake.\n"
  exit 1
fi

printf "\nBuilding and Installing:\n"
pushd $DELPHYNE_BUILD_DIR
make -j$( getconf _NPROCESSORS_ONLN )

##################################### CPP ####################################

printf "\nRunning C++ tests:\n"
CTEST_OUTPUT_ON_FAILURE=1 make test || CPP_EXIT=$?
popd

################################### PYTHON ###################################

export PYTHONPATH=$DELPHYNE_INSTALL_DIR/lib/python2.7/site-packages:$DELPHYNE_INSTALL_DIR/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DELPHYNE_INSTALL_DIR/lib

printf "\nRunning Python tests:\n"
pushd $DELPHYNE_BUILD_DIR/python
python setup.py egg_info --egg-base `pwd` sdist test || PYTHON_EXIT=$?
popd

################################### RESULT ###################################
# Output results and consolidate exit result to 0, 1.

printf "\n********************** Results **********************\n\n"

if [ "$CPP_EXIT" -ne 0 ]; then
  printf " - Some c++ tests failed.\n"
else
  printf " - All c++ tests passed.\n"
fi

if [ "$PYTHON_EXIT" -ne 0 ]; then
  printf " - Some python tests failed.\n"
else
  printf " - All python tests passed.\n"
fi
printf "\n"
EXIT=0
[ "$CPP_EXIT" -ne 0 ] || [ "$PYTHON_EXIT" -ne 0 ] && EXIT=1
exit $EXIT

