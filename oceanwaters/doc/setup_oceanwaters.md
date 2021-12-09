These instructions cover the environment setup, installation, and build of
OceanWATERS. Familiarity with Unix/Linux and the Bash shell command line is assumed.

If you have not yet done so, [install the prerequisites for OceanWATERS](setup_dev_env.md).

ROS workspace setup
-------------------

Installation of OceanWATERS requires creation of a ROS *workspace* – a directory
where you can build multiple, interdependent ROS packages
([*http://wiki.ros.org/catkin/workspaces*](http://wiki.ros.org/catkin/workspaces)).
ROS provides custom tools, *catkin* and *wstool*, to manage, build, and install
ROS packages in a workspace. [Catkin](https://wiki.ros.org/catkin) is a build system that sits on top of
the CMake build system ([*https://cmake.org*](https://cmake.org)) and wstool is
a meta–version control system tool that allows you to manage multiple source
repositories.

Reading [Catkin documentation](https://wiki.ros.org/catkin) and going through some of
the [tutorials](https://wiki.ros.org/catkin/Tutorials) is recommended if you've
never used catkin.

wstool is quite straightforward and can be picked up easily. Basically, wstool
will look at a .rosinstall file that contains the repos to check out or update.
You can perform operations like update, diff, and status on multiple
repositories at once.

To create a ROS workspace for OceanWATERS first use the setup.bash script
provided in the ROS installation to set required ROS environment variables:

```
source /opt/ros/noetic/setup.bash
```

Next create a workspace directory for OceanWATERS with a `src` subdirectory, and
go to the `src` subdirectory:

```
mkdir -p oceanwaters_ws/src
cd oceanwaters_ws/src
```

Then checkout the top level OceanWATERS git repository:

```
git clone https://github.com/nasa/ow_simulator.git
```

ROS’ wstool will be used to checkout additional repositories containing the
source code files making up OceanWATERS. The wstool utility will look for a
file, `oceanwaters_ws/src/.rosinstall`, that contains a list of repositories to
check out or update. The OceanWATERS repository contains a number of `.rosinstall`
files for different uses. Create a link to the default repository list:

```
ln -s ow_simulator/oceanwaters/workspaces/oceanwaters.rosinstall .rosinstall
```

Now use wstool to checkout/update the source code:

```
wstool update
```

NOTE: At this point you will have cloned the `master` branch of all OceanWATERS
repositories. `master` branch is a stable snapshot of `noetic-devel` branch,
both compatible with ROS Noetic on Ubuntu 20.04. 


OceanWATERS Build
-----------------

To build the workspace, cd back up to the workspace directory and run catkin build.

```
cd ..
catkin build
```

The first build of OceanWATERS will take an extra long time as the build process downloads
around 2 GB of 3rd party code and data. The code and data will be cached in the src/ tree,
so subsequent builds should be much faster.

Finally set OceanWATERS specific environment variables using the provided
script:

```
source oceanwaters_ws/devel/setup.bash
```

At this point you should see Gazebo related variables such as
`GAZEBO_MODEL_PATH` set in your environment.

Launch the simulator to make sure that the build was successful:

```
roslaunch ow europa_terminator_workspace.launch
```

Many windows will appear, as well as many messages in the terminal (including
some warnings and errors) which at this time are expected.  But when startup
finishes, you should see the RViz window showing a 3D model of the lander, and
the Gazebo window also showing the terrain.

Congratulations, your OceanWATERS installation and setup is complete!
