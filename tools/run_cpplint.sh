#!/bin/bash

find . -not \( -path ./bridge/drake -prune \) -not \( -path ./bridge/protobuf -prune \) -iname '*.cc' -o -iname '*.cpp' -o -iname '*.c' -o -iname '*.hpp' -o -iname '*.hh' -o -iname '*.h' | while read file ; do
    python tools/cpplint.py --extensions=cc,cpp,c,hh,hpp,h --headers=h,hh,hpp "$file"
done
