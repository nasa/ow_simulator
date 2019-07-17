ow_lander
=========
Contains the lander semantic and kinematic descriptions (SRDF and URDF), as well
as the moveit path planner and the gazebo trajectory feeder.

To run along with Gazebo:

`$ roslaunch ow_lander ow_arm_sim.launch`

This will start Gazebo, as well as the planning and feeding nodes. Now to run
both these nodes, you need to call the appropriate services (note that PLEXIL
will be calling these services in code):

Full trajectory planning
------------------------
Call the StartPlanning service. Args:
```
bool use_defaults        # Set to true to use the defualt trench loc
float32 trench_x         # Trench x location
float32 trench_y         # Trench Y location
float32 trench_d         # Trench depth
bool delete_prev_traj    # Set to true to cleanup ~/.ros
```

move_guarded trajectory planning
--------------------------------
Call the MoveGuarded service. This creates two trajectories in .ros Args:
```
bool use_defaults           # Set to true to use the default touch location
bool delete_prev_traj       # Set to true for cleanup
float32 target_x            # Touch x,y,z
float32 target_y
float32 target_z
float32 surface_normal_x    # Normal vector (for approach direction)
float32 surface_normal_y
float32 surface_normal_z
float32 offset_distance     # Start offset dist along normal vector
float32 overdrive_distance  # How deep after expected ground do we go
float32 retract             # TODO, unused as of now
```

Trajectory feeding
------------------
Call the PublishTrajectory service. Args:
```
bool use_latest             # Set to true to use the latest trajectory 
string trajectory_filename  # The filename of the trajectory to run
```

Manual Operations
-----------------
To run these services manually, run 

`$ rqt`

Then, in the plugins menu, add a Service Caller to the window. Then, from the
drop down menu, select the service you want (`/planning/start_planning_session`
or `/planning/publish_trajectory`), set the arguments (Careful, the bools have a
capital first letter: `True`, `False`), then call the service.

