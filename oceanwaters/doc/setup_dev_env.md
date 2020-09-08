Requirements
============

Hardware
--------

The minimum recommended hardware features are as follows:
* 2.0 GHz quad core x86-64 CPU
* 16 GB memory
* Nvidia graphics card. GeForce GTX 750-Ti, GeForce GTX 960M, Quadro M1200, or better.

Software
--------

The following software is required to build and run OceanWATERS:
* Operating System: Linux Ubuntu 18.04 (bionic) specifically. (Even slightly
newer versions such as 18.10 may not work.)
* The Bash command shell
* Robot Operating System (ROS), melodic version specifically, including:
* Gazebo simulator 9.13
* catkin build system
* PLEXIL plan language and executive (http://plexil.sourceforge.net).

Note on virtual machines (e.g. VMWare, Parallels, VirtualBox, Windows Subsystem
for Linux): These have all worked to some degree, but tend to not support the
Gazebo simulator very well or at all. They are not recommended for OceanWATERS.

Development Environment Setup
=================

ROS and Gazebo
------------

* Install ROS (Melodic version) by following the
[instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). Select the
ros-melodic-desktop-full package when you get to that step.

* Install Gazebo 9.13+. ROS melodic ships with Gazebo 9.0 which does not satisfy
OceanWATERS requirements. To get the latest stable version of Gazebo available
to ROS melodic follow these steps: 

  * First run `gazebo --version` and check the version that is currently installed,
if you have 9.13 or higher installed then you may skip this Gazebo upgrade.

  * Add OSRF gazebo repositories to your linux enviroment:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

  * Update and upgrade the packages (confirm the upgrade when you get prompted):
```
sudo apt-get update
sudo apt-get upgrade
```
  * Run `gazebo --version` and verify that you have a version of 9.13 or higher

* In addition to the above OceanWATERS requires the following list of packages:
```
sudo apt install git \
                 python-wstool \
                 python-catkin-tools \
                 ros-melodic-tf2-ros \
                 ros-melodic-robot-state-publisher \
                 ros-melodic-joint-state-publisher \
                 ros-melodic-joint-state-controller \
                 ros-melodic-effort-controllers \
                 ros-melodic-dynamic-reconfigure \
                 ros-melodic-nodelet \
                 ros-melodic-nodelet-topic-tools \
                 ros-melodic-camera-info-manager \
                 ros-melodic-tf2-geometry-msgs \
                 ros-melodic-gazebo-ros-control \
                 ros-melodic-xacro ros-melodic-rviz-visual-tools \
                 ros-melodic-rqt-plot ros-melodic-rqt-rviz \
                 ros-melodic-rqt-image-view \
                 ros-melodic-rqt-common-plugins \
                 ros-melodic-gazebo-plugins \
                 ros-melodic-moveit \
                 ros-melodic-moveit-ros-visualization \
                 ros-melodic-geometry-msgs \
                 ros-melodic-cmake-modules \
                 ros-melodic-stereo-msgs \
                 ros-melodic-stereo-image-proc \
                 libgtk2.0-dev \
                 libglew-dev
```

PLEXIL
------

PLEXIL is hosted on sourceforge.net, which provides both source code and binary
distributions. Because a variety of machines are used in this project, it is
best built from the source code, which must be checked out of its git
repository. This has the added advantage of allowing customization of the
installation.

* Check out the source code:
```
git clone https://git.code.sf.net/p/plexil/git plexil
```

NOTE: the default branch is releases/plexil-4, which is the latest stable version of PLEXIL.

* Install any of the following build prerequisites needed. If you're not sure,
try the build, see where it breaks, and install new packages as you go. All of
the following may be installed with: `sudo apt install <package-name>`
```
sudo apt install make \
                 autotools-dev \
                 autoconf \
                 libtool \
                 g++ \
                 ant \
                 gperf \
                 openjdk-8-jdk
```

* Add the following lines to your ~/.bashrc file, using your actual location of
plexil in the first line:
```
export PLEXIL_HOME=/home/<username>/plexil
source $PLEXIL_HOME/scripts/plexil-setup.sh
```

* Source your shell init file.
```
source ~/.bashrc
```

* Configure for the build as needed for OceanWATERS.
```
cd $PLEXIL_HOME
make src/configure
cd src
./configure CFLAGS="-g -O2" CXXFLAGS="-g -O2" --prefix=$PLEXIL_HOME --disable-static --disable-viewer --enable-ipc
```

* Build it.
```
cd $PLEXIL_HOME
make
```

* If you have problems, see additional build information
[here](http://plexil.sourceforge.net/wiki/index.php/Installation).

