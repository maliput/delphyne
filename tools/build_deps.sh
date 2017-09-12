#!/bin/bash -xe

TOOLS='ign_tools ign_math ign_common ign_msgs ign_transport ign_gui ign_rendering'

PROCS=`getconf _NPROCESSORS_ONLN`

mkdir -p deps
pushd deps
vcs import < ../delphyne.repos

for tool in $TOOLS ; do
    pushd $tool
    mkdir build
    pushd build
    cmake .. -DCMAKE_INSTALL_PREFIX=../../installed
    make -j$PROCS install
    popd
    popd
done

popd
