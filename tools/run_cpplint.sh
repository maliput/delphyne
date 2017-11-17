#!/bin/bash

python tools/cpplint.py \
    --quiet \
    --recursive \
    --extensions=cc,h \
    $( find . -iname '*.cc' -o -iname '*.h' )
