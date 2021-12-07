The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

ow_faults
=========
This package contains infrastructure for fault management and injection. Users
can inject simulated faults into their simulations to cause problems for their
autonomy algorithms.

Faults are implemented using ROS parameters.

## `faults injection` node
The `faults` node in this package is the source of all the fault state in
OceanWATERS. It uses a `dynamic_reconfigure::Server` under the hood so that
clients can modify fault state at runtime.

## Setting faults
You can set faults in rqt, on the command line, or from a python script.
Setting a fault causes it to be updated on the `dynamic_reconfigure::Server` in
the faults node such that a message is emitted on the `/faults/update_parameters`
topic and `rqt_reconfigure` is updated with your new value.

### rqt
In rqt, look at the `faults` section in the `Dynamic Reconfigure` widget. There
you can see and set all faults available in OceanWATERS.

### Command line
To see all possible faults you can set, do this:

`rosparam list /faults`

To set a particular fault:

`rosrun dynamic_reconfigure dynparam set /faults ant_pan_joint_locked_failure True`

Here is more information about [dynparam](http://wiki.ros.org/dynamic_reconfigure#dynamic_reconfigure.2BAC8-groovy.dynparam_command-line_tool).

Note: you *cannot* use `rosparam` to set faults:

`rosparam set /faults/ant_pan_joint_locked_failure True`

That will set the param on the `faults` node, but it won't be changed in the
node's `dynamic_reconfigure::Server` and the change won't propagate to the rest
of the simulation.

### Python script
Include the dynamic_reconfigure client API:

`import dynamic_reconfigure.client`

Then set faults on the `faults` node from anywhere in your script:

```
client = dynamic_reconfigure.client.Client('/faults')
params = { 'ant_pan_joint_locked_failure' : 'True'}
config = client.update_configuration(params)
```

## Simulation of faults
Some faults are easier to simulate than others. For example, a dead sensor can
be simulated by turning off its output or setting its output to zero. But a
problem with the movement of a joint must be simulated by modifying the
properties of the joint in the dynamics simulation.

The `faults` node currently handles the simplest of these faults. Joint encoder
and torque sensor failures are simulated by receiving `/joint_states` and
publishing a modified version of it called `/faults/joint_states`. Similarly,
all faulty versions of messages should be prefixed with `/faults`.

As more faults are added, their simulation will need to be distributed
throughout the OceanWATERS components depending on their characteristics.

