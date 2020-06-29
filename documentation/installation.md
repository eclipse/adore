<!--
********************************************************************************
* Copyright (C) 2017-2020 German Aerospace Center (DLR). 
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the 
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0 
*
* Contributors: 
*   Daniel Heß 
********************************************************************************
-->


# Installation and Getting Started

This guide describes the necessary steps to install the adore framework for development.

## Setup

It is assumed an installation on Linux (Ubuntu 18.04) with Visual Studio Code and ROS Melodic is carried out.

### ROS installation

To install ROS melodic execute these lines in a shell:

~~~bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop python-catkin-tools
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
~~~

For further details see the [ROS melodic installation guide](http://wiki.ros.org/melodic/Installation/Ubuntu)

### Visual Studio Code

We recommend the use of Visual Studio Code as the default source code editor for this project.

Download vscode from <https://code.visualstudio.com/download> and then execute this in a shell:

~~~bash
sudo apt install ./\<filename\>.deb
~~~

We advise to install these extensions from within vscode (Ctrl + Shift + X)

* C/C++ from Microsoft
* ROS from Microsoft
* doxdocgen

### System Libraries

To install all system dependencies at once, execute the following command from a shell:

~~~bash
sudo apt-get install libboost-dev libboost-date-time-dev libboost-system-dev libboost-chrono-dev libboost-thread-dev libboost-signals-dev libblas-dev libopenblas-dev liblapack-dev freeglut3-dev libxerces-c-dev  libcurl4-openssl-dev xterm doxygen git apt-utils lsb-release gnupg wget software-properties-common unzip wget xsd psmisc
~~~

### Install a new version of cmake:

* CMake > 3.13 - [install as package](https://apt.kitware.com/) - our reasons for this minimum version can be found [here](http://dominikberner.ch/cmake-interface-lib/) and [here](https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/)

~~~bash
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt-get update
sudo apt-get install cmake
~~~

### Git setup

You might want to setup Git before going on. Hints and details can be found in the [Git guide](git_guide.md).

### Folders and Git cloning

ROS needs a workspace folder, we recommend `~/catkin_ws` as default.
Execute these lines in a shell to create a workspace and clone the repository:

~~~bash
mkdir -p ~/catkin_ws/src/adore
cd ~/catkin_ws/src
git clone https://github.com/eclipse//adore.git adore
~~~

### (Optional) Setup ccache

The build optimzer tool ccache helps to speed up compilation in cases where unchanged files are compiled again. This could happen after a catkin clean for example. The tool will intercept compiler calls by having the symbolic links of the compiler point to the tool instead. For compiler calls where ccache already has an artifact in the cache it will return this from cache. In all other circumstances ccache will forward the reuqest to the actual compiler.

More details can be found on the [ccache Github page](https://github.com/ccache/ccache) including instructions on how to [build and setup ccache](https://github.com/ccache/ccache/blob/master/doc/INSTALL.md).

### Additional Python dependencies

To run python nodes the following dependencies need to be installed. For the advanced user it is advised to consider virtualenv or conda instead of a direct installation.

~~~bash
sudo apt-get install python3-pip
pip3 install pyyaml
pip3 install rospkg
~~~

## Setup adore, test build

During the first build, you should be connected to the internet, as several required libraries are downloaded and installed in subdirectories of adore/src.
Subsequent builds will not download or rebuild required libraries, while the automatically created folders exist.
To build, execute the following steps in a shell:

~~~bash
cd ~/catkin_ws/
catkin config --install
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build adore_if_ros
~~~

if you get no error messages, the setup is complete :-)

## Test ADORe simulation

If the following lines are executed, a plotlab figure should display a roadmap loaded from an exemplary OpenDrive file.

~~~bash
source ~/catkin_ws/install/setup.bash
cd ~/catkin_ws/src/adore/plotlabserver
./start.sh
cd ~/catkin_ws/src/adore/adore_if_ros_demos
roslaunch demo001_loadmap.launch
~~~

For further information and demos see [adore_if_ros_demos](../adore_if_ros_demos/README.md).
