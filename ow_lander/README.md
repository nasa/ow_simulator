===== ow_lander =====
Contains the lander semantic and kinematic descriptions (SRDF and URDF), as well as the moveit path planner and the gazebo trajectory feeder

=== Trajectory planning === TEMPORARY: will be slave to plexil
To plan a digging trajectory run:
 
 $ roslaunch ow_lander plan_trajectory.launch

One can override the default trench location and depth with trench_x:=<value>, trench_y:=<value> and trench_d:=<value>

=== Trajectory feeding === TEMPORARY: will be slave to plexil
To feed the trajectory into a running gazebo lander sim, run:

 $ rosrun ow_lander send_moveit_trajectory.bash

 This will publish the latest trajectory from the ~/.ros/ folder on the 6 arm joint command topics


