#!/bin/bash

python tools/cpplint.py \
    --quiet \
    --recursive \
    --extensions=cc,h \
    $( find . -not \( -path ./test/gtest -prune \) -iname '*.cc' -o -iname '*.h' )
