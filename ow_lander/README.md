===== ow_lander =====
Contains the lander semantic and kinematic descriptions (SRDF and URDF), as well as the moveit path planner and the gazebo trajectory feeder

To run along with Gazebo

$ roslaunch ow_lander ow_arm_sim.launch

This will start Gazebo, as well as the planning and feeding nodes. Now to run both these nodes, you need to call the appropriate services (note that PLEXIL will be calling these services in code):

=== Trajectory planning === 
Call the StartPlanning service. Args:

bool use_defaults 			# Set to true to use the defualt trench loc
float32 trench_x			# Trench x location
float32 trench_y			# Trench Y location
float32 trench_d			# Trench depth
bool delete_prev_traj		# Set to true to cleanup ~/.ros

=== Trajectory feeding === 
Call the PublishTrajectory service. Args:

bool use_latest				# Set to true to use the latest trajectory 
string trajectory_filename	# The filename of the trajectory to run

=== Manual Operations === 
To run these services manually, run 

$ rqt

Then, in the plugins menu, add a Service Caller to the window. Then, from the drop down menu, select the service you want (/planning/start_planning_session or /planning/publish_trajectory), set the arguments (Careful, the bools have a capital first letter: True, False), then call the service