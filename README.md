[![GCC](https://github.com/maliput/delphyne/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/delphyne/actions/workflows/build.yml)

# delphyne
## Description

Delphyne is a traffic level simulator for autonomous driving.

Road network model is provided by [`Maliput`](https://maliput.readthedocs.io/en/latest/index.html) libraries.

Related packages:
 - [delphyne_gui](https://github.com/maliput/delphyne_gui): Provides a visualizer for `delphyne`.
 - [delphyne_demos](https://github.com/maliput/delphyne_demos): Provides demos for `delphyne`.

## API Documentation

Refer to [Delphyne's Online API Documentation](https://maliput.readthedocs.io/en/latest/html/deps/delphyne/html/index.html).

## Examples

Visit [delphyne_demos](https://github.com/maliput/delphyne_demos).

## Installation

### Supported platforms

Ubuntu Focal Fossa 20.04 LTS.

### Source Installation on Ubuntu

#### Prerequisites

```
sudo apt install python3-rosdep python3-colcon-common-extensions
```

#### Build

1. Create colcon workspace if you don't have one yet.
    ```sh
    mkdir colcon_ws/src -p
    ```

2. Clone dependencies in the `src` folder
    ```sh
    cd colcon_ws/src
    git clone https://github.com/maliput/drake_vendor.git
    ```

3. Clone this repository in the `src` folder
    ```sh
    cd colcon_ws/src
    git clone https://github.com/maliput/delphyne.git
    ```

4. Install package dependencies via `rosdep`
    ```
    export ROS_DISTRO=foxy
    ```
    ```sh
    rosdep update
    rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths src
    ```
5. Follow instructions to install drake via [`drake_vendor`](https://github.com/maliput/drake_vendor) package.

6. Build the package
    ```sh
    colcon build --packages-up-to delphyne
    ```

    **Note**: To build documentation a `-BUILD_DOCS` cmake flag is required:
    ```sh
    colcon build --packages-select delphyne --cmake-args " -DBUILD_DOCS=On"
    ```
    More info at [Building Documentation](https://maliput.readthedocs.io/en/latest/developer_guidelines.html#building-the-documentation).

For further info refer to [Source Installation on Ubuntu](https://maliput.readthedocs.io/en/latest/installation.html#source-installation-on-ubuntu)


### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/maliput/delphyne/blob/main/LICENSE)
