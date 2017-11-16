#!/bin/bash

python tools/cpplint.py --extensions=c,cc,cpp,cxx,c++,C,h,hh,hpp,hxx,inc --headers=h,hh,hpp,hxx --recursive $( find . -iname '*.cc' -o -iname '*.cpp' -o -iname '*.c' -o -iname '*.hpp' -o -iname '*.hh' -o -iname '*.h' )
