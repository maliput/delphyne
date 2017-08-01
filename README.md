# Drakaina

This is the repository for Drakaina.  As of right now, the only supported platform is Ubuntu 16.04 amd64.

# Setup instructions

1.  You first need to install the dependencies for Drake.  While Drakaina doesn't directly depend on Drake right now, it may in the future and the build system aims to be compatible.  The instructions are [here](http://drake.mit.edu/from_source.html), but in brief:

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
$ sudo apt-get install mercurial cmake pkg-config python ruby-ronn libprotoc-dev libprotobuf-dev protobuf-compiler uuid-dev libzmq3-dev git libignition-transport3-dev libogre-1.9-dev libglew-dev qtbase5-dev libicu-dev libboost-filesystem-dev libfreeimage-dev libtinyxml2-dev libgts-dev libavdevice-dev libopencv-calib3d-dev libopencv-contrib-dev libopencv-gpu-dev libopencv-legacy-dev libopencv-ocl-dev libopencv-photo-dev libopencv-stitching-dev libopencv-superres-dev libopencv-ts-dev libopencv-videostab-dev libavcodec-dev libopencv-dev libasound-dev libopenal-dev libwebp-dev libwavpack-dev libvpx-dev libjack-dev libcdio-dev libva-dev libxvidcore-dev libzvbi-dev libx264-dev libx265-dev libcdio-cdda-dev libcdio-paranoia-dev libxv-dev libsdl-dev libvorbis-dev libsnappy-dev libspeex-dev libtwolame-dev libssh-dev libsoxr-dev libtheora-dev libshine-dev libcrystalhd-dev libgnutls-dev libiec61883-dev libavc1394-dev libass-dev libbluray-dev libbs2b-dev libgme-dev libgsm1-dev libmodplug-dev librtmp-dev libmp3lame-dev libopus-dev libopenjpeg-dev libschroedinger-dev flite1-dev libbz2-dev
```

# Repository structure
There are 2 things hosted here right now; the bridge from LCM messages to ignition-transport messages, and the new drake visualizer.  The bridge is stored in the bridge/ subdirectory, and the visualizer
is stored in the visualizer/ subdirectory.

# Building
To build the bridge, run:

```
$ bazel build //bridge:*
```

To build the visualizer, run:

```
$ bazel build //visualizer:*
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
