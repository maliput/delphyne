# Delphyne

This is the repository for Delphyne - a traffic level simulator for autonomous driving.

## Build

1. Setup a development workspace as described [here](https://github.com/ToyotaResearchInstitute/dsim-repos-index/tree/master/setup/README.md).

2. Bring up your development workspace:

```sh
cd path/to/my/workspace
source ./bringup
```

3. Build Delphyne packages and their dependencies:

  - If not building drake from source:

   ```sh
   colcon build --packages-up-to delphyne-gui
   ```

  - If building drake from source:

   ```sh
   colcon build --cmake-args -DWITH_PYTHON_VERSION=3 --packages-up-to delphyne-gui
   ```

Note: If you want to build tests, make sure to add `--cmake-args -DBUILD_TESTS=ON` to colcon build. Example:
```sh
   colcon build --cmake-args -DBUILD_TESTS=ON --packages-up-to delphyne-gui
```

## Use

1. Bring up your development workspace:

```sh
cd path/to/my/workspace
source ./bringup
```

2. Source your build:

```sh
source ./install/setup.bash
```

3. Run any demo, for instance:

```sh
delphyne-gazoo
```

You can checkout any other example by typing delphyne and pressing TAB to trigger autocompletion.

## Learn and Contribute

User and developer instructions can be found in the
[Delphyne Guide](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ).

* [Delphyne Guide#Development](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ/edit#heading=h.lemp4kd4zn0j)

If submitting pull requests pay particular attention to the relevant sections in [Delphyne Guide#Development](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ/edit#heading=h.lemp4kd4zn0j).
