> The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

This directory contains ROS Action server scripts, as well as client
scripts to test the actions.

Note that the action server scripts are run automatically when the
simulator is started using any of the provided launch files.  The
action client scripts are test scripts to be run interactively.  When
invoked with no arguments, they use defaults that succinctly
demonstrate their operation.  To see a list of the positional
arguments they can be called with, along with their default values,
invoke the script with just a `-h` or `--help` option.  If you invoke
the script with a partial argument list supplied, defaults will be
used for the remainder.

NOTE: the default values used in the action client test scripts may
work only with the Atacama terrain, as started with

```bash
roslaunch ow atacama_y1a.launch
```

Assuming your terminal is in this directory, e.g.

```bash
cd ow_simulator/ow_lander/scripts
```

each individual arm action can be invoked as follows.  These examples
will use the default arguments.

```bash
./unstow_action_client.py
```
```bash
./guarded_move_action_client.py
```
```bash
./grind_action_client.py
```
```bash
./dig_circular_action_client.py
```
```bash
./dig_linear_action_client.py
```
```bash
./deliver_action_client.py
```
```bash
./stow_action_client.py
```
