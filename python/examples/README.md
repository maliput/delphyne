# Python examples

These examples are intended to demonstrate how to use the python bindings of the
simulation runner.

For all the examples below it's assumed that the Delphyne backend was successfully
built with CMake, as shown in the [Delphyne Guide](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ).
<h1 id="all_cars_in_dragway">all_cars_in_dragway</h1>


This demo consists of a suite of dynamically loaded cars,
running simultaneously on a dragway road.
For the time being, three cars are supported:
   - A Prius Simple Car.
   - A MOBIL Simple Car.
   - A MaliputRailCar.

<h1 id="keyboard_controlled_simulation">keyboard_controlled_simulation</h1>


This example shows how to use the keyboard events to control the advance of
a simulation. The simulation will open the usual simple car in the center of
the scene, which can be driven using the keyboard on the GUI's teleop widget.
However, by switching to the console, we can `play`/`pause`/`step`/`quit` the
simulation.

```
$ cd <delphyne_ws>/install/bin
$ ./keyboard_controlled_simulation.py
```

 The supported keys for the demo:

<`p`> will pause the simulation if running and vice-versa.

<`s`> will step the simulation once if paused.

<`q`> will stop the simulation and quit the demo.

<h1 id="maliput_racing_circuit">maliput_racing_circuit</h1>


An example of three railcars and a variable number of MOBIL controlled car
running in a closed-loop maliput road.

In order to run the demo with a number of MOBIL cars different to three, a number
between 0 and 6 can be passed as an argument. For example:

```
maliput_racing_circuit.py --num-cars=2
```

<h1 id="realtime_rate_changer">realtime_rate_changer</h1>


This example shows how the real-time simulation rate can be set both when the
simulator runner is created and while the simulation is running.

To pass an initial real-time rate use the `--realtime_rate` flag, like:

```
$ realtime_rate_changer.py --realtime_rate=2.0
```

If none is specified the default will be set to `1.0` (i.e. run the simulation
in real-time).

Once the scripts starts running it will cycle between a real-time rate of `0.6`
to `1.6` to depict how dynamic real-time rate impacts on the simulation.

<h1 id="road_loading">road_loading</h1>


This example shows how to run a simulation that includes a road. For the
time being three road examples are supported: dragway, onramp, monolane and
multilane.
This demo uses the subcommand style, where each road type can handle different
parameters (to list the available arguments just do
`$ road_loading <road_type> -h`). Below are some examples of usage:


A dragway that is 200 meters long and has a side-shoulder of 2.5 meters:

```
$ road_loading.py dragway --length=200 --shoulder-width=2.5
```

An on-ramp road:

```
$ road_loading.py onramp
```

Load a monolane file:

```
$ road_loading.py monolane
--filename='./install/share/delphyne/roads/double_ring.yaml'
```

Load a multilane file:

```
$ road_loading.py multilane
--filename='./install/share/delphyne/roads/circuit.yaml'
```


<h1 id="simple_python_binding">simple_python_binding</h1>

This is a minimal example of starting an automotive simulation using a
python binding to the C++ `SimulatorRunner` class.

Note that this is not a configurable demo, it will just create a sample
simulation with a prius car that can be driven around.

Check the other examples in this directory for more advanced uses.

<h1 id="start_simulation_paused">start_simulation_paused</h1>


This example show how to use the `SimulationRunner`'s (optional) third
constructor argument to start in pause mode.

```
$ cd <delphyne_ws>/install/bin
$ ./start_simulation_paused.py
```

This command will spawn a visualizer instance and start the simulation in
paused mode.

 ## Unpausing the simulation

- The most user-friendly way of unpausing the simulation is by using the
`TimePanel` widget in the visualizer. This panel allows to control the
simulation with `Play` / `Pause` / `Step` buttons from the GUI.

- Through the command line, by using the `WorldControl` service, publishing a
message with the right content into the `/world_control` channel:

```
$ cd <delphyne_ws>/install/bin
$ ./ign service --service /world_control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 500 --req 'pause: false'
```


<h1 id="time_bounded_simulation">time_bounded_simulation</h1>


This example shows how to run a simulation for a fixed period of time (15
seconds by default). As an added bonus, the user can also specify the real-time
simulation rate (which by default is 1.0).

The following command depicts how to run a simulation for 30 sim seconds using
a real-time rate of 2x (so it should run in approximately 15 wall clock
seconds):

```
$ time_bounded_simulation.py --realtime_rate=2.0 --duration=30.0
```

