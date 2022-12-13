These instructions cover system requirements, and the installation of
prerequisite software (ROS, PLEXIL, GSAP), for OceanWATERS.  Familiarity with
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

-   The Ubuntu 20.04 LTS (Focal Fossa) Linux operating system, with:
    -   The Bash command shell
    -   The git version control system
-   [ROS Noetic Ninjemys](http://wiki.ros.org/noetic) distribution, with:
    -   The Catkin build system
-   Gazebo 11.12+
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

The OceanWATERS distribution includes an autonomy module
(`ow_autonomy`) that at present uses PLEXIL, an open-source plan
authoring language and autonomy executive (see
[*http://plexil.sourceforge.net](http://plexil.sourceforge.net)).
PLEXIL must be installed and built *prior* to building the `ow_plexil`
package.

Note that both source code and binary distributions of PLEXIL are
available at the Sourceforge link above. However, only the source code
distribution should be used, because the binaries are out of date and
will not be compatible with OceanWATERS.

* Check out the `releases/plexil-4.6` branch of the source code:
```
git clone --branch releases/plexil-4.6 https://git.code.sf.net/p/plexil/git plexil
```

* Install any of the following build prerequisites needed. If you're
not sure which, if any, are missing on your system, try the build as
instructed below, and if there are errors that indicate missing
software packages, install them and try again.  All packages needed
can be installed with: `sudo apt install <package-name>`.  Here is one
command to get them all:

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

* Define the `PLEXIL_HOME` environment variable as the location of
  your PLEXIL installation.  This is needed _prior_ to building
  PLEXIL, and will be needed each time you run PLEXIL, with
  OceanWATERS or otherwise.  Therefore we recommend placing the
  following command (substituting the correct pathname) in your shell
  initialization file, e.g. `~/.bashrc`.

```
export PLEXIL_HOME=/home/<username>/plexil
```

* Source the PLEXIL initialization file.  This will also be needed
  every time you run PLEXIL, so we recommend placing this in your
  shell initialization file as well, below the previous command.

```
source $PLEXIL_HOME/scripts/plexil-setup.sh
```

* Configure PLEXIL for the build.

NOTE: OceanWATERS must be with a version of this branch tagged
`OceanWATERS-v10.0`.  (Its git commit hash begins with `cde7d07`).
Newer versions of this branch do not build at the time of this
writing.  Soon, a new distribution of PLEXIL on GitHub will be made
official.

```
cd $PLEXIL_HOME
git checkout OceanWATERS-v10.0
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

Installation of OceanWATERS requires prior installation of ROS Noetic. ROS
binary downloads for Ubuntu Linux are available at
[http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation),
and instructions for installing ROS are available at
[http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu).

* Install ROS (Noetic version) by following
[these instructions](http://wiki.ros.org/noetic/Installation/Ubuntu). Select the
ros-noetic-desktop-full package when you get to that step.

In the remainder of this document, we assume that ROS has been installed under `/opt/ros/noetic`.

### Gazebo

* Install Gazebo 11.12+.

  * First run `gazebo --version` and check the version that is currently installed,
if you have 11.12 or higher installed then you may skip this Gazebo upgrade.

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
  * Run `gazebo --version` and verify that you have a version of 11.12 or higher.

  * If you may be using Gazebo/OceanWATERS with a VPN (Virtual Private
    Network) running, it is highly recommended you add the following
    line to your shell initialization file (e.g. .bashrc).  An
    explanation of the issue can be found in the [troubleshooting
    guide](https://github.com/nasa/ow_simulator/wiki/Troubleshooting#ignition-transport-failure-on-vpn).

```
export IGN_IP=127.0.0.1
```



### Additional Packages

* In addition to the above, OceanWATERS requires packages listed in the
  following installation command.

```
sudo apt install git \
                 python3-wstool \
                 python3-catkin-tools \
                 ros-noetic-tf2-ros \
                 ros-noetic-robot-state-publisher \
                 ros-noetic-joint-state-publisher \
                 ros-noetic-joint-state-controller \
                 ros-noetic-effort-controllers \
                 ros-noetic-joint-trajectory-controller \
                 ros-noetic-dynamic-reconfigure \
                 ros-noetic-nodelet \
                 ros-noetic-nodelet-topic-tools \
                 ros-noetic-camera-info-manager \
                 ros-noetic-tf2-geometry-msgs \
                 ros-noetic-gazebo-ros-control \
                 ros-noetic-xacro \
                 ros-noetic-rviz-visual-tools \
                 ros-noetic-rqt-plot \
                 ros-noetic-rqt-rviz \
                 ros-noetic-rqt-image-view \
                 ros-noetic-rqt-common-plugins \
                 ros-noetic-gazebo-plugins \
                 ros-noetic-moveit \
                 ros-noetic-moveit-commander \
                 ros-noetic-moveit-ros-visualization \
                 ros-noetic-geometry-msgs \
                 ros-noetic-cmake-modules \
                 ros-noetic-stereo-msgs \
                 ros-noetic-stereo-image-proc \
                 ros-noetic-kdl-parser-py \
                 libgtk2.0-dev \
                 libglew-dev
```
