#!/bin/bash
export PATH=$PATH:/home/$USER/.local/bin

check_program_installed() {
  if ! [ -x "$(command -v $1)" ]; then
    printf "Error: %s was not found in your system.\n" "$1"
    printf "Please install it by running:\n\n"
    printf "pip install --user %s\n\n" "$1"
    exit 1
  fi
}

check_program_installed pycodestyle
check_program_installed pylint

declare -i PEP8FAILED=0
declare -i PYLINTFAILED=0

# Exclude Dirs:
#  - build/style helper scripts in ./tools
#  - test helper scripts in test/utils
#  - entry points in python/examples

# Run PEP 8
grep -rl --exclude-dir={tools,utils,examples} '^#!/.*python' . | xargs pycodestyle ||  PEP8FAILED=1
if [ "$PEP8FAILED" -eq "0" ]; then
  # Run pylint
grep -rl --exclude-dir={tools,utils,examples} '^#!/.*python' . | xargs pylint || PYLINTFAILED=1
else
  echo $'\n*** PEP8 failed, not doing pylint ***'
  exit 1
fi

if [ "$PYLINTFAILED" -ne "0" ]; then
  echo $'\n*** Pylint failed ***'
  exit 1
fi

