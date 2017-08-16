# Delphyne

This is the repository for Delphyne.  As of right now, the only supported platform is Ubuntu 16.04 amd64.

# Setup instructions

1.  You first need to install the dependencies for Drake.  While Delphyne doesn't directly depend on Drake right now, it may in the future and the build system aims to be compatible.  The instructions are [here](http://drake.mit.edu/from_source.html), but in brief:

    ```
    $ git clone git@github.com:RobotLocomotion/drake.git drake-distro
    $ cd drake-distro
    $ sudo ./setup/ubuntu/16.04/install_prereqs.sh
    ```

1. Next you need to add the gazebo repositories to your setup.  The canonical instructions are [here](https://ignition-transport.readthedocs.io/en/latest/installation/installation.html#ubuntu-linux), in brief:

    ```
    $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    $ sudo apt-get update
    ```

1. Now you can install additional dependencies

    ```
    $ sudo apt-get install mercurial cmake pkg-config python ruby-ronn libprotoc-dev libprotobuf-dev protobuf-compiler uuid-dev libzmq3-dev git libignition-transport3-dev libogre-1.9-dev libglew-dev qtbase5-dev libicu-dev libboost-filesystem-dev libfreeimage-dev libtinyxml2-dev libgts-dev libavdevice-dev
    ```

# Repository structure
There are 2 things hosted here right now; the bridge from LCM messages to ignition-transport messages, and the new drake visualizer.  The bridge is stored in the bridge/ subdirectory, and the visualizer
is stored in the visualizer/ subdirectory.

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
