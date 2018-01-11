# Python examples

These examples are intended to demonstrate how to use the python bindings of the simulation runner.

For all the examples below it's assumed that the Delphyne backend was successfully built with CMake,
as shown in the instructions [here](https://github.com/ToyotaResearchInstitute/delphyne-gui/blob/master/README.md#build-delphyne-back-end).

# paused_mode_example.py

This example shows the instantiation of a SimulationRunner making use of the an optional argument, that enables the simulator to start in paused mode.

```
$ cd <delphyne_ws>/install/bin
$ ./paused_mode_example.py --paused
```

This command will spawn a visualizer instance and start the simulation in paused mode.

## Unpausing the simulation
This task can be achieved in two different ways.

- The first way involves making use of a WorldControl service, by publishing a
WorldControl message into the /world_control channel with the right command.

```
$ cd <delphyne_ws>/install/bin
$ ./ign service --service /world_control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 500 --req 'pause: false'
```

- The second way is still unavailable, but worth mentioning here since it's aimed
to be the default way of controlling the simulation once it's ready.
It involves making use of the TimePanel widget in the visualizer (currently under
development), which should enable the user to control the simulation with buttons
like Play, Pause, Step, all from the Visualizer's GUI.


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
