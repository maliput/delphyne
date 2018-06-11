#!/bin/bash

declare -i PEP8FAILED=0
declare -i PYLINTFAILED=0

# Run PEP 8
export PATH=$PATH:/home/$USER/.local/bin
grep -rl --exclude-dir={tools,utils} '^#!/.*python' . | xargs pycodestyle ||  PEP8FAILED=1

if [ "$PEP8FAILED" -eq "0" ]; then
  # Run pylint
  grep -rl --exclude-dir={tools,utils} '^#!/.*python' . | xargs pylint || PYLINTFAILED=1
else
  echo $'\n*** PEP8 failed, not doing pylint ***'
  exit 1
fi

if [ "$PYLINTFAILED" -ne "0" ]; then
  echo $'\n*** Pylint failed ***'
  exit 1
fi

