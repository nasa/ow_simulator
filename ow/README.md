This project contains top-level launch files for ow_simulator.

arm_sim.launch : by default this starts Gazebo.  To suppress Gazebo, e.g. in virtual machines where Gazebo does not run well, use the 'gazebo' argument set to false:

  % roslaunch ow arm_sim.launch gazebo:=false

