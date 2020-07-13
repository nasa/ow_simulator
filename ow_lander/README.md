The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

ow_lander
=========
Contains the lander semantic and kinematic descriptions (SRDF and URDF), the
moveit path planner and the gazebo trajectory feeder, and common code related
to the lander.

Common Code
-----------
`include/ow_lander/lander_joints.h` contains joint names as they appear in the
URDF and related code. Please prefer this header over writing similar code
yourself to keep all our code consistent. If the lander is modified to change
the number of joints or any joint names, this header must be updated as well.

Run it
------
To run along with Gazebo:

`$ roslaunch ow europa_terminator_workspace.launch`

This will start Gazebo, as well as the planning and feeding nodes. Now to run
both these nodes, you need to call the appropriate services (note that PLEXIL
will be calling these services in code):

### Full trajectory planning
Call the DigCircular service. Args:
```
bool use_defaults        # Set to true to use the defualt trench loc
float32 trench_x         # Trench x location
float32 trench_y         # Trench Y location
float32 trench_d         # Trench depth
bool delete_prev_traj    # Set to true to cleanup ~/.ros
```

### guarded_move trajectory planning
Call the GuardedMove service. This creates two trajectories in .ros Args:
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

### Trajectory feeding
Call the PublishTrajectory service. Args:
```
bool use_latest             # Set to true to use the latest trajectory 
string trajectory_filename  # The filename of the trajectory to run
```

### Manual Operations
To run these services manually, find the 'rqt' window and select the Service
Caller tab. Then, from the drop down menu, select the service you want
(`/planning/arm/dig_circular` or `/planning/arm/publish_trajectory`), set the
arguments (Careful, the bools have a capital first letter: `True`, `False`),
then call the service.

