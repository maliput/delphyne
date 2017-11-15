# Delphyne

This is the repository for Delphyne. As of right now, the only supported
platform is Ubuntu 16.04 amd64.

# Setup instructions

Please, follow the instructions detailed
[here](https://github.com/ToyotaResearchInstitute/delphyne-gui) for
building the dependencies and the Delphyne project.

# Developer's section
This section is aimed to show the scripts and tools available on delphyne for the developers.

## Instructions for the clang-format tool
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

This script must be run from the top-level of the repository in order to find
all of the files. It is recommended to run this before opening any pull request.

## Instructions for the cpplint tool
In order to get the project's source code analyzed by cpplint, you can run a wrapper script that goes through the whole project looking for errors.

```
./tools/run_cpplint.sh
```

This command will run the tool and print the results to console.

## Instructions for running code-coverage tools
Delphyne includes a third party CMake tool aimed to ease the use of code-coverage tools, which enable us to run them from a simple make command.

In order to make use of this tools, you need to pass the special parameter `CMAKE_BUILD_TYPE=coverage` to CMake, and then build the project as usual.

Make sure to run `make test` before `make coverage`, it'll fail otherwise.
```
$ cd <path/to/delphyne/build>
$ cmake ../../src/delphyne -DCMAKE_INSTALL_PREFIX=../../install -DCMAKE_BUILD_TYPE=coverage
$ make
$ make test
$ make coverage
```

In order to see the web report, run:
```
$ firefox coverage/index.html
```

If you want simple coverage numbers in plain text, you can inspect the following files:
```
$ cat coverage/lines.txt
$ cat coverage/functions.txt
```

Which will show the percentage of the project's lines of code and functions covered by the tests, respectively.
