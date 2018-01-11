# Python examples

These examples are intended to demonstrate how to use the python bindings of the simulation runner.

For all the examples below it's assumed that the Delphyne backend was successfully built with CMake,
as shown in the instructions [here](https://github.com/ToyotaResearchInstitute/delphyne-gui/blob/master/README.md#build-delphyne-back-end).

# keyboard_controlled_simulation.py
This example shows how to use the keyboard events to control the advance of a simulation.

```
$ cd <delphyne_ws>/install/bin
$ ./keyboard_controlled_simulation.py
```

 The supported keys for the demo:

<`p`> will pause the simulation if unpaused and viceversa.

<`s`> will step the simulation once if paused.

<`q`> will stop the simulation and quit the demo.
