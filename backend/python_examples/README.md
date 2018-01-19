# Python examples

These examples are intended to demonstrate how to use the python bindings of the simulation runner.

For all the examples below it's assumed that the Delphyne backend was successfully built with CMake,
as shown in the instructions [here](https://github.com/ToyotaResearchInstitute/delphyne-gui/blob/master/README.md#build-delphyne-back-end).

# paused_mode_example.py

This example show how to use the `SimulationRunner`'s (optional) third constructor argument to start in pause mode.

```
$ cd <delphyne_ws>/install/bin
$ ./paused_mode_example.py
```

This command will spawn a visualizer instance and start the simulation in paused mode.

## Unpausing the simulation

- At this moment, the only available way of achieving this is by making use of a `WorldControl` service, publishing a message with the right content into the `/world_control` channel:

```
$ cd <delphyne_ws>/install/bin
$ ./ign service --service /world_control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 500 --req 'pause: false'
```

- An alternative way will be to use the `TimePanel` widget in the visualizer (currently under development), which will allow to control the simulation with Play / Pause / Step buttons from the GUI.

# keyboard_controlled_simulation.py
This example shows how to use the keyboard events to control the advance of a simulation. The simulation will open the usual simple car
in the center of the scene, which can be driven using the keyboard on the GUI's teleop widget. However, by switching to the console, we
can play/pause/step/quit the simulation.

```
$ cd <delphyne_ws>/install/bin
$ ./keyboard_controlled_simulation.py
```

 The supported keys for the demo:

<`p`> will pause the simulation if running and vice-versa.

<`s`> will step the simulation once if paused.

<`q`> will stop the simulation and quit the demo.