#!/bin/bash

SCRIPT_PATH=$(realpath ${BASH_SOURCE[0]})
SCRIPT_DIR=$(dirname $SCRIPT_PATH)
REPO_DIR=$SCRIPT_DIR/..

python $SCRIPT_DIR/cpplint.py \
    --quiet \
    --recursive \
    --extensions=cc,h \
    $( find $REPO_DIR -iname '*.cc' -o -iname '*.h' )
