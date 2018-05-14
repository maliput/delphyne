#!/bin/bash

declare -i PEP8FAILED=0

# Run PEP 8
find ./python -name \*.py -exec pycodestyle {} + || PEP8FAILED+=1

 if [ "$PEP8FAILED" -eq "0" ]; then
  # Run pylint
  find ./python -name \*.py -exec pylint {} +
else
   echo $'\n*** PEP8 failed, not doing pylint ***'
fi

