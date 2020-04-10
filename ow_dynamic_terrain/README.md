
# introduction
A demo of dynamic terrain within ROS/Gazebo environment

![](./misc/scene_dynamic_terrain_no_smooth_1.gif)

# requirements

ROS distro: melodic  

Required Packages:
  - [ros-melodic-husky-desktop](https://wiki.ros.org/husky_desktop)
  - [ros-melodic-husky-simulator](http://wiki.ros.org/husky_gazebo)
  - [ros-melodic-teleop-twist-keyboard](http://wiki.ros.org/teleop_twist_keyboard)

> **_NOTE:_** The DynamicTerrainModel plugin relies on a recently merged pull request https://bitbucket.org/osrf/gazebo/pull-requests/3210/add-setheight-method-to-heightmapshape. So make sure to link against that latest gazebo9 branch if you wish to compile the code.

# usage
Launch demo world using ```roslaunch dynamic_terrain terrain_world.launch```.  
To control the husky robot using a keyboard execute ```rosrun teleop_twist_keyboard teleop_twist_keyboard.py``` in another terminal.
