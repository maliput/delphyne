#!/bin/bash

SCRIPT_PATH=$(realpath ${BASH_SOURCE[0]})
SCRIPT_DIR=$(dirname $SCRIPT_PATH)

python $SCRIPT_DIR/cpplint.py \
    --quiet \
    --recursive \
    --extensions=cc,h \
    $( find . -iname '*.cc' -o -iname '*.h' )
