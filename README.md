# Delphyne

This is the repository for Delphyne.  As of right now, the only supported platform is Ubuntu 16.04 amd64.

# Setup instructions

1.  You first need to install the dependencies for Drake.  While Delphyne doesn't directly depend on Drake right now, it may in the future and the build system aims to be compatible.  The instructions are [here](http://drake.mit.edu/from_source.html), but in brief:

    ```
    $ git clone git@github.com:RobotLocomotion/drake.git drake-distro
    $ cd drake-distro
    $ sudo ./setup/ubuntu/16.04/install_prereqs.sh
    ```

1. Next you need to setup ROS dependencies:

    ```
    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    $ sudo apt-get update
    ```

1. Now you can install additional dependencies

    ```
    $ sudo apt-get install mercurial cmake pkg-config python ruby-ronn libprotoc-dev libprotobuf-dev protobuf-compiler uuid-dev libzmq3-dev git libogre-1.9-dev libglew-dev qtbase5-dev libicu-dev libboost-filesystem-dev libfreeimage-dev libtinyxml2-dev libgts-dev libavdevice-dev python3-vcstool
    ```

# Build dependencies

Delphyne depends on a number of external dependencies.  To make the tools and libraries easy to use, we build them from source and install them into the workspace.  The best way to do this is to run the following script from the top of the delphyne directory:

```
$ tools/build_deps.sh
```

This may take a little while to download and build the dependencies.  At the end of the build, a new subdirectory called `deps` will be at the top level of your delphyne repository.

# Setup your environment
In order to successfully build and use tools here, a few environment variables must be setup.  We recommend putting these in ~/.bashrc (or the equivalent for your shell):

```
$ export DELPHYNE_MEDIA_PATH=</absolute/path/to/delphyne/media>:</path/to/drake-distro>
$ export PKG_CONFIG_PATH=</absolute/path/to/delphyne>/deps/installed/lib/pkgconfig
$ export PATH=</absolute/path/to/delphyne>/deps/installed/bin
$ export LD_LIBRARY_PATH=</absolute/path/to/delphyne>/deps/installed/lib
```

# Repository structure
There are 2 things hosted here right now; the bridge from LCM messages to ignition-transport messages, and the new drake visualizer.  The bridge is stored in the `bridge/` subdirectory, and the visualizer is stored in the `visualizer/` subdirectory.

# Building
To build everything, run:

```
$ bazel build //...
```

# Running
To run the bridge, run:

```
$ bazel run //bridge:lcm-to-ign-trans-bridge
```

To run the visualizer, run:

```
$ bazel run //visualizer:visualizer
```

To run the visualizer & bridge bundle, run:

```
$ export DELPHYNE_PACKAGE_PATH=</path/to/drake-distro/drake/automotive/models>
$ bazel run //bridge:mocked-robot-demo
```

To run the demo-launcher, run:
```
$ bazel run //bridge:demo-launcher </path/to/drake-distro>
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