# Delphyne

This is the repository for Delphyne.  As of right now, the only supported platform is Ubuntu 16.04 amd64.

# Setup instructions

1.  Setup and install dependencies:

    ```
    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    $ sudo apt-get update
    $ sudo apt-get install mercurial cmake pkg-config python ruby-ronn libprotoc-dev libprotobuf-dev protobuf-compiler uuid-dev libzmq3-dev git libogre-1.9-dev libglew-dev qtbase5-dev libicu-dev libboost-filesystem-dev libfreeimage-dev libtinyxml2-dev libgts-dev libavdevice-dev python3-vcstool mesa-utils
    ```

1.  Now build a workspace for Delphyne.  If you are familiar with ROS catkin workspaces, this is a similar concept.  The steps to setup the workspace are:

    ```
    $ mkdir -p delphyne_ws/src
    $ cd delphyne_ws
    ```

1.  Download the delphyne.repos file from [delphyne.repos](https://github.com/ToyotaResearchInstitute/delphyne/delphyne.repos) into `delphyne_ws` (we can't automate this because it is a private repository).

1.  Import the repositories from delphyne.repos:

    ```
    $ vcs import src < delphyne.repos
    ```

1.  Install the dependencies for Drake.  While Delphyne doesn't directly depend on Drake right now, it may in the future and it depends on many similar components.  The instructions are [here](http://drake.mit.edu/from_source.html), but in brief:

    ```
    $ sudo ./src/drake/setup/ubuntu/16.04/install_prereqs.sh
    ```

# Setup the environment
In order to successfully build and use Delphyne, Drake, or the ignition tools here, a few environment variables must be setup.  These have to be absolute paths, so note that the following will only work from the root of the delphyne workspace:

```
$ export PKG_CONFIG_PATH=`pwd`/install/lib/pkgconfig:$PKG_CONFIG_PATH
$ export PATH=`pwd`/install/bin:$PATH
$ export LD_LIBRARY_PATH=`pwd`/install/lib:$LD_LIBRARY_PATH
$ export DELPHYNE_PACKAGE_PATH=`pwd`/src/drake/drake/automotive/models:`pwd`/src/delphyne/bridge
```

Next we can go on and build the components.

# Build dependencies

Delphyne depends on a number of external dependencies.  To make the tools and libraries easy to develop with, we build them from source and install them into the workspace.  Right now this is done manually, by running the following commands:

```
$ mkdir -p build
$ pushd build
$ for igndep in ign_tools ign_math ign_common ign_msgs ign_transport ign_gui ign_rendering; do mkdir -p $igndep ; pushd $igndep ; cmake ../../src/$igndep -DCMAKE_INSTALL_PREFIX=../../install ; make -j$( getconf _NPROCESSORS_ONLN ) install ; popd ; done
$ popd
```

This may take a little while to build the dependencies.  At the end of the build, a new subdirectory called `install` will be at the top level of your delphyne workspace.

# Build drake

Drake must be built using the Bazel build tool.  To build drake, do the following:

```
$ pushd src/drake
$ bazel build //...
$ popd
```

Note that this will take a long time to compile.

# Build delphyne

Delphyne itself can now be built with Bazel:

```
$ pushd src/delphyne
$ bazel build //...
$ popd
```

# Running

Two different things are hosted in the delphyne repository right now; the bridge and the visualizer.  The bridge is a bi-directional LCM-to-ignition-transport to take messages from/to Drake and translate them to/from ignition-transport messagesthat Delphyne understands.  The visualizer is a new front-end visualizer for the drake simulator.

## Running the mock demo

There is a mock demo hosted in this repository that runs the bridge, the visualizer, and a "mock" backend that simulates the messages that Drake would send.  To run this demo, do the following:

```
$ pushd src/delphyne
$ bazel run //bridge:mocked-robot-demo
$ popd
```

## Running the bridge, visualizer, and drake

There is a demonstration here of using Delphyne together with the automotive_demos from drake.  To run this demo, do the following:

```
$ pushd src/delphyne
$ bazel run //bridge:demo-launcher </path/to/drake-distro>
$ popd
```
Please note that the export paths used here and for the mocked-robot-demo are the same, and should point to drake's "models" directory.

The launcher includes three predefined demos that you can specify using the `--demo` parameter, like:
```
$ bazel run //bridge:demo-launcher -- --demo=simple </path/to/drake-distro>
```

the possible values for the predefined demos are: simple, trajectory and dragway.

Finally, you can append the `--no-drake-visualizer` argument option to the command, which will disable the drake visualizer, as well as the lcm-spy and lcm-logger:

```
$ bazel run //bridge:demo-launcher -- --demo=simple --no-drake-visualizer </path/to/drake-distro>
```

## Running the bridge standalone

To run just the bridge, run:

```
$ pushd src/delphyne
$ bazel run //bridge:lcm-ign-transport-bridge
$ popd
```

## Running the visualizer standalone:

To run just the visualizer, run:

```
$ pushd src/delphyne
$ bazel run //visualizer:visualizer
$ popd
```

# Instructions for the clang-format tool
In order to get all the C++ code in the project compliant with a single style, we strongly recommend you using the auto-formatting tool called clang-format.

You can execute it against your source code by doing:
```
/usr/bin/clang-format-3.9 -i -style=file <path/to/file.cpp>
```
This will automatically apply the code conventions specified in the .clang-format file, found on the root of the repository.

There is also an automated script that looks for all the C++ src/header files and then calls clang-format accordingly. You can invoque it by doing:

```
./tools/reformat_code.sh
```

This script must be run from the top-level of the repository in order to find all of the files. It is recommended to run this before opening any pull request.
