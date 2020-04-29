
# introduction
A demo of dynamic terrain within ROS/Gazebo environment

![](./misc/scene_dynamic_terrain_no_smooth_1.gif)

# requirements

ROS distro: melodic  

> **_NOTE:_** The DynamicTerrainModel plugin relies on a recently merged pull request https://bitbucket.org/osrf/gazebo/pull-requests/3210/add-setheight-method-to-heightmapshape. So make sure to link against that latest gazebo9 branch if you wish to compile the code.

# usage
Launch demo world using ```roslaunch ow_dynamic_terrain europa.launch```.  

Then you may perform a terrain operation by submitting a rostopic message as follows:

```
rostopic pub /ow_dynamic_terrain/modify_terrain ow_dynamic_terrain/modify_terrain "{operation: lower, position: {x: 0,  y: 0}, outer_radius: 0.1, inner_radius: 0.001, weight: 1}"
```

Current supported terrain operations are: raise, lower, flatten, smooth