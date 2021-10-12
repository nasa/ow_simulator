These instructions cover system requirements, and the installation of
prerequisite software (PLEXIL, ROS), for OceanWATERS.  Familiarity with
Unix/Linux and the Bash shell command line is assumed.

System Requirements
-------------------

### Hardware

The minimum recommended hardware configuration is as follows:

-   2.0 GHz quad core x86-64 CPU
-   16 GB memory
-   Nvidia graphics card. GeForce GTX 750-Ti, GeForce GTX 960M, Quadro M1200, or
    better.

### Software

The following minimum software set is required to build and run OceanWATERS
(version numbers and identifiers are listed where specific versions are
required):

-   The Ubuntu 18.04 (Bionic) Linux operating system, with:
    -   The Bash command shell
    -   The git version control system
-   The Robot Operating System (ROS), Melodic Morenia distribution, with:
    -   The Catkin build system
-   Gazebo 9.13
- PLEXIL plan language and executive (http://plexil.sourceforge.net).
- Generic Software Architecture for Prognostics (GSAP) v2.0

IMPORTANT NOTE: Virtual machines (in particular VMWare, Parallels, VirtualBox,
Windows Subsystem for Linux v2) have all worked to some degree, but tend to not
support the Gazebo simulator very well or at all. *Virtual machines are not
recommended for OceanWATERS.*


Prerequisites
-------------

OceanWATERS requires PLEXIL, GSAP, ROS, and Gazebo.
In the following instructions, we assume the default command shell is Bash.

### PLEXIL

The OceanWATERS distribution includes an autonomy module (`ow_autonomy`) that at
present uses PLEXIL, an open-source plan authoring language and autonomy
executive (see [*http://plexil.sourceforge.net](http://plexil.sourceforge.net)).
PLEXIL must be installed and built *prior* to building the `ow_autonomy`
package.

Note that both source code and binary distributions of PLEXIL are available at
the Sourceforge link above. However, only the source code distribution should be
used with OceanWATERS, because the binaries are out of date or might not be
compatible.


* Check out the source code:
```
git clone https://git.code.sf.net/p/plexil/git plexil
```

The default git branch of PLEXIL is `releases/plexil-4`, which is maintained as
a stable version of PLEXIL compatible with OceanWATERS and suitable for general
use.

NOTE: At the time of this writing, OceanWATERS has been tested with a version of
this branch tagged `OceanWATERS-v7.1`.  Its git commit hash begins with `df7ed1e`.
Newer versions of this branch should work with the newest version of the
`master` branches of OceanWATERS.

* Install any of the following build prerequisites needed. If you're not sure
which, if any, are missing on your system, try the build, and if there are
errors that indicate missing software packages, install them and try again.  All
packages needed can be installed with: `sudo apt install <package-name>`.
Here's one command to get them all.

```
sudo apt install make \
                 autotools-dev \
                 autoconf \
                 libtool \
                 g++ \
                 ant \
                 gperf \
                 default-jre
```

Note that PLEXIL (specifically the plan compiler) requires the Java compiler and
runtime environment, version 8 or newer.

* Define the `PLEXIL_HOME` environment variable as the location of your PLEXIL
  installation, e.g.

```
export PLEXIL_HOME=/home/<username>/plexil
```

* Source the PLEXIL initialization file:
```
source $PLEXIL_HOME/scripts/plexil-setup.sh
```

NOTE: for convenience, you may wish to add the previous two commands to your
shell initialization file (e.g. `.profile`), since they are needed every time
you use PLEXIL or the `ow_autonomy` package.

* Configure PLEXIL for the build:
```
cd $PLEXIL_HOME
make src/configure
cd src
./configure CFLAGS="-g -O2" CXXFLAGS="-g -O2" --prefix=$PLEXIL_HOME --disable-static --disable-viewer --enable-ipc
```

* Build PLEXIL.
```
cd $PLEXIL_HOME
make universalExec plexil-compiler checkpoint
```

* Note that this is a minimal build of PLEXIL including only what is needed by
OceanWATERS.  Additional build information is available
[here](http://plexil.sourceforge.net/wiki/index.php/Installation).

* Rebuiding PLEXIL.

At a later date, if you update (e.g. `git pull`) your PLEXIL installation, it is
safest to rebuild it from scratch:
```
cd $PLEXIL_HOME
make squeaky-clean
make universalExec plexil-compiler checkpoint
```

### GSAP

The OceanWATERS distribution includes a power system module (`ow_power_system`)
that at present depends on GSAP, an open-source battery prognostics
executive. GSAP must be installed *prior* to building the `ow_power_system`
package.

* Check out the source code:
```
git clone -b v1.0-OW https://github.com/nasa/GSAP.git gsap
```

This checks out the git _tag_ `v1.0-OW` of GSAP's `master` branch, which is a
tested version of GSAP for use with OceanWATERS.  Note that checking out a
specific tag leaves your local repository in a "detached HEAD" state; this is of
no concern.

* Define the `GSAP_HOME` environment variable as the location of your GSAP
  installation, e.g.

```
export GSAP_HOME=/home/<username>/gsap
```

NOTE: for convenience, you may wish to add the previous command to your shell
initialization file (e.g. `.profile` or `.bashrc`), since they are needed every
time.

* Build GSAP.
```
cd $GSAP_HOME
mkdir build
cd build
cmake ..
make
```

NOTE: Configuration files can be used to tune the prognostics algorithm and/or
adjust the prognostics model used to perform calculations.  An example
configuration file can be found at: ``` ow_power_system/config/example.cfg ```
Additional information about mapping configuration files and modifying their
contents can be found [here](https://github.com/nasa/GSAP/wiki/Getting-Started).

* If you have problems, see additional build information
[here](https://github.com/nasa/GSAP/wiki).

### ROS

Installation of OceanWATERS requires prior installation of ROS Melodic. ROS
binary downloads for Ubuntu Linux are available at
[http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation),
and instructions for installing ROS are available at
[http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu).

* Install ROS (Melodic version) by following
[these instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). Select the
ros-melodic-desktop-full package when you get to that step.

By default, ROS is installed in `/opt/ros/release`. In the remainder of this document,
we assume that ROS is installed in `/opt/ros/melodic`.

### Gazebo

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

### Additional Packages

* In addition to the above, OceanWATERS requires packages listed in the
  following installation command.

```
sudo apt install git \
                 python-wstool \
                 python-catkin-tools \
                 ros-melodic-tf2-ros \
                 ros-melodic-robot-state-publisher \
                 ros-melodic-joint-state-publisher \
                 ros-melodic-joint-state-controller \
                 ros-melodic-effort-controllers \
                 ros-melodic-joint-trajectory-controller \
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
                 ros-melodic-kdl-parser-py \
                 libgtk2.0-dev \
                 libglew-dev
```
