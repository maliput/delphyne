# Delphyne

This is the repository for Delphyne. As of right now, the only supported
platform is Ubuntu 16.04 amd64.

# Documentation

User and developer instructions can be found in the
[Delphyne Guide](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ].

* [Delphyne Guide#Installation](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ/edit#heading=h.e45b0m13gxl4)
* [Delphyne Guide#Development](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ/edit#heading=h.lemp4kd4zn0j)

If submitting PR's pay particular attention to the relevant sections in [Delphyne Guide#Development](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ/edit#heading=h.lemp4kd4zn0j).


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
