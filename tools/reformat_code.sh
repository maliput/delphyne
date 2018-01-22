#!/bin/bash

# This is a simple script to run the clang-format against all of the code
# in the repository.  clang-format will reformat all of the code according
# to the format described in the top-level .clang-format file in this
# repository.  This script must be run from the top-level of the repository
# in order to find all of the files.  It is recommended to run this before
# opening any pull request.

find . -not \( -path ./bridge/drake -prune \) \
  -not \( -path ./bridge/protobuf -prune \) \
  -not \( -path ./test -prune \) \
  -not \( -path ./backend/linb-any -prune \) \
  -not \( -path ./backend/test/linb_any_test.cc \) \
  -iname '*.cc' -o -iname '*.cpp' -o -iname '*.c' -o -iname '*.hpp' -o -iname '*.hh' -o -iname '*.h' | while read file ; do
    echo "Reformatting $file"
    clang-format-3.9 -i -style=file "$file"
done
