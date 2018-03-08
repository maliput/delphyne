# Delphyne

This is the repository for Delphyne. As of right now, the only supported
platform is Ubuntu 16.04 amd64.

# Instructions

Instructions for setting up a delphyne workspace can be found in [Delphyne Guide#Installation](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ/edit#heading=h.e45b0m13gxl4).

# Developer's section
This section is aimed to show the scripts and tools available on delphyne for the developers.

## Instructions for the clang-format tool
In order to get all the C++ code in the project compliant with a single style, we strongly recommend you using the auto-formatting tool called clang-format.

You can execute it against your source code by doing:
```
/usr/bin/clang-format-3.9 -i -style=file <path/to/file.cpp>
```
This will automatically apply the code conventions specified in the .clang-format file, found on the root of the repository.

There is also an automated script that looks for all the C++ src/header files and then calls clang-format accordingly. You can invoke it by doing:

```
./tools/reformat_code.sh
```

This script must be run from the top-level of the repository in order to find
all of the files. It is recommended to run this before opening any pull request.

## Instructions for the cpplint tool
In order to get the project's source code analyzed by cpplint, you can run a wrapper script that goes through the whole project looking for errors.

```
./tools/run_cpplint.sh
```

This command will run the tool and print the results to console.

## Instructions for running tests
All the available tests, both C++ and Python, can be run by calling a single helper script. It can be invoked as follows:
```
./tools/run_tests.sh
```
