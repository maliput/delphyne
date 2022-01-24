| GCC | Sanitizers(Clang) | Scan-Build |
| --------- | --------- | -------- |
|[![gcc](https://github.com/ToyotaResearchInstitute/delphyne/actions/workflows/build.yml/badge.svg)](https://github.com/ToyotaResearchInstitute/delphyne/actions/workflows/build.yml) | [![clang](https://github.com/ToyotaResearchInstitute/delphyne/actions/workflows/sanitizers.yml/badge.svg)](https://github.com/ToyotaResearchInstitute/delphyne/actions/workflows/sanitizers.yml) | [![scan_build](https://github.com/ToyotaResearchInstitute/delphyne/actions/workflows/scan_build.yml/badge.svg)](https://github.com/ToyotaResearchInstitute/delphyne/actions/workflows/scan_build.yml) |

# Delphyne

This is the repository for Delphyne - a traffic level simulator for autonomous driving.

## Build

1. Setup a development workspace as described [here](https://github.com/ToyotaResearchInstitute/maliput_documentation/blob/main/docs/installation_quickstart.rst).

2. Build Delphyne packages and their dependencies:

  - If not building drake from source:

   ```sh
   colcon build --packages-up-to delphyne_demos
   ```

  - If building drake from source:

   ```sh
   colcon build --cmake-args -DWITH_PYTHON_VERSION=3 --packages-up-to delphyne_demos
   ```

## Use

1. Source your build:

```sh
source ./install/setup.bash
```

3. Run any demo, for instance:

```sh
delphyne_gazoo
```

You can checkout any other example by typing delphyne and pressing TAB to trigger autocompletion.

## Learn and Contribute

User and developer instructions can be found in the
[Delphyne Guide](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ).

* [Delphyne Guide#Development](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ/edit#heading=h.6wa9ubx28pkv)

If submitting pull requests pay particular attention to the relevant sections in [Delphyne Guide#Development](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ/edit#heading=h.6wa9ubx28pkv).
