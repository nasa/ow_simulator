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


### ROS Action for ARM Joint Movements

In accordance to Command Unification document specifications with JPL testbed a new ROS action interface has been added. This is called ArmMoveJoint which allows an user to move any particular joint. The ROS action client accepts three arguments, Relative, Joint Index and Joint Angle. Relative is a bool argument which when set true move a particular joint from the current position to a relative position specified in Joint index and joint angle. Joint index is from 0 to 5 wiht tht the follwing Joints indices. 0:j_shou_yaw, 1:j_shou_pitch, 2:j_prox_pitch, 3:j_dist_pitch, 4:j_hand_yaw, 5:j_scoop_yaw. Joint angle is specified in Radians

```bash
./arm_move_joint_client.py -h or help
```

The ArmMoveJoint client can be called by the following command for default (pre-defined in the client). 
```bash
./arm_move_joint_client.py
```
To move a specific joint to a  particular angle (in radians) use  : 
```bash
python3 arm_move_joint_client.py False 0 0.5
```

To move a joint to a relative position from the current position use: 
```bash
python3 arm_move_joint_clienet.py True 0 0.5
```

Note: Commanding the arm using this command can cause collision with the lander body and/or terrain. Some motions will be denied by the motion planner (like self collision of the arm) but not all. Use this command with extreme caution as this may break the simulation. 
