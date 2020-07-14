The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

ow_simulator
============
This project contains top-level launch files for ow_simulator.

arm_sim.launch : by default this starts Gazebo.  To suppress Gazebo, e.g. in virtual machines where Gazebo does not run well, use the 'gazebo' argument set to false:

  % roslaunch ow arm_sim.launch gazebo:=false

