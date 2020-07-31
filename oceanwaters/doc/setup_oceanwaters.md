If you have not yet done so, [set up your development environment](setup_dev_env.md).

Setup OceanWATERS
=================
ROS Workspaces
--------------
ROS uses custom tools called Catkin and wstool to manage workspaces. Catkin is a
build system that sits on top of CMake, and wstool is a meta-vcs tool that
allows you to manage multiple source repositories. Reading
[Catkin documentation](https://wiki.ros.org/catkin) and going through some of
the [tutorials](https://wiki.ros.org/catkin/Tutorials) is recommended if you've
never used it before.

wstool is quite straightforward and can be picked up easily. Basically, wstool
will look at a .rosinstall file that contains the repos to check out or update.
You can perform operations like update, diff, and status on multiple
repositories at once.

Create OceanWATERS Workspace
----------------------------
* Create your workspace
```
mkdir -p oceanwaters_ws/src
cd oceanwaters_ws/src
```
* Next, check out the top level oceanwaters repo which contains the default
.rosinstall workspaces
```
git clone https://github.com/nasa/ow_simulator
```
* Symlink the .rosinstall you're interested in into the src directory and run
wstool. This will check out all the repos necessary to build the workspace.
```
ln -s oceanwaters/workspaces/oceanwaters_v1.rosinstall .rosinstall
wstool up
```
* To build the workspace, cd back up to the workspace directory, source the ROS
environment, and run catkin build. The first time you build should take an extra
long time as the build process downloads around 2 GB of 3rd party code and data.
The code and data will be cached in the src/ tree, so subsequent builds should
be much faster.
```
cd ..
source /opt/ros/melodic/setup.bash
catkin build
```
* NOTE: if catkin is not found, it was somehow left out of your ROS install.  Try:
```
sudo apt-get install python-catkin-tools
```
* The workspace should now be compiled and ready to run. Source the workspace
environment.
```
source devel/setup.bash
```
* Launch the sim
```
roslaunch ow europa_terminator_workspace.launch
```

