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
*   Thomas Lobig
********************************************************************************
-->

# Installation and Getting Started

This guide describes the necessary steps to install the adore framework for development.

## Setup

The given instructions assume an installation of ROS Noetic on Linux (Ubuntu 20.04) While the middleware-agnostic part libadore can be built without ROS, it is highly recommended to install ROS to benefit from the full framework and the demos.

## Requirements

Python 3.5–3.8 is required for running python nodes and installing dependencies. Note that tensorflow does currently
not support Python 3.9

## WSL

The adore framework can also be installed in an wsl 1 environment without special handling. Note that wsl 2 should
also work, but this is untested. To display plotlab figures for demo, an Xserver like Xming is required on the host.

Just install Xming and export the display variable in the wsl environment.

~~~bash
echo "export DISPLAY=:0" >> ~/.bashrc
~~~

Visual Studio codes comes with a Remote development extension, which is suitable for developing inside the wsl environment.
More information on this topic can be found here: <https://code.visualstudio.com/docs/remote/wsl>

### ROS installation

To install ROS Noetic execute these lines in a shell:

~~~bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
~~~

To build the ROS-specific parts of ADORe the build tool catkin is needed. The tool and further ROS-specific dependencies can be installed by running these lines:

~~~bash
sudo apt-get install python3-pip python3-catkin-tools python3-roslaunch git
sudo pip3 install osrf-pycommon
~~~

For further details see the [ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu)

The tool rosdep is optional with ADORe but helpful for ROS development. It can be installed by running the following lines

~~~bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
~~~

### Visual Studio Code

We recommend using Visual Studio Code as the default source code editor for this project.

There are two options for Ubuntu 20.04:

* Download vscode from <https://code.visualstudio.com/download> and then execute this in a shell:

    ~~~bash
    sudo apt install ./\<filename\>.deb
    ~~~

* Use snap to install vscode:

    ~~~bash
    sudo snap install --classic code
    ~~~

We advise to install these extensions from within vscode (Ctrl + Shift + X)

* C/C++ from Microsoft
* ROS from Microsoft
* doxdocgen

### System Libraries

To install all system dependencies at once, execute the following command from a shell:

~~~bash
sudo apt-get install libboost-dev libboost-date-time-dev libboost-system-dev libboost-chrono-dev libboost-thread-dev libblas-dev libopenblas-dev liblapack-dev freeglut3-dev libxerces-c-dev  libcurl4-openssl-dev xterm doxygen git apt-utils lsb-release gnupg wget software-properties-common unzip wget xsdcxx psmisc libprotobuf-dev protobuf-compiler libdlib-dev
~~~

### ZeroMQ

To build and install the libzmq library and the cppzmq C++ binding, execute the commands below from a shell. Note: Choose a directory outside the catkin workspace for cloning or remove the cloned files after installation.

~~~bash
# choose a directory outside the catkin workspace to clone the files to
git clone https://github.com/zeromq/libzmq.git
cd libzmq
mkdir build
cd build
cmake ..
sudo make -j4 install
cd ../..
git clone https://github.com/zeromq/cppzmq.git
cd cppzmq
mkdir build
cd build
cmake -DENABLE_DRAFTS=ON ..
sudo make -j4 install
~~~

For further details see the [cppzmq README](https://github.com/zeromq/cppzmq)

### Git setup

We recommend to setup Git before continuing. Hints and details can be found in the [Git guide](git_guide.md).

### (Optional) Setup ccache

The build optimzer tool ccache helps to speed up compilation in cases where unchanged files are compiled again. This could happen after a catkin clean for example. The tool will intercept compiler calls by having the symbolic links of the compiler point to the tool instead. For compiler calls where ccache already has an artifact in the cache it will return this from cache. In all other circumstances ccache will forward the reuqest to the actual compiler.

More details can be found on the [ccache Github page](https://github.com/ccache/ccache) including instructions on how to [build and setup ccache](https://github.com/ccache/ccache/blob/master/doc/INSTALL.md).

### Additional Python dependencies

To run python nodes the following dependencies need to be installed.

~~~bash
pip3 install pyyaml
pip3 install rospkg
pip3 install tensorflow
pip3 install pydot
pip3 install graphviz
pip3 install tf-agents
pip3 install pycryptodomex
pip3 install gnupg
pip3 install requests
pip3 install -v --pre pyzmq --install-option=--enable-drafts
~~~

## Setup ADORe, test build

ROS needs a workspace folder, we recommend `~/catkin_ws` as default.
Execute these lines in a shell to create a workspace and clone the repository:

~~~bash
mkdir -p ~/catkin_ws/src/adore
cd ~/catkin_ws/src
git clone https://github.com/eclipse/adore.git adore
~~~

During the first build, a connection to the internet is required. Several required libraries are downloaded to subdirectories of adore/src or catkin_ws/build/_deps.
Subsequent builds will not download or rebuild required libraries, while the automatically created folders exist.
To build, execute the following steps in a shell:

~~~bash
cd ~/catkin_ws/
catkin config --install
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF
catkin build adore_if_ros
~~~

If there are no error messages, the setup is complete.


## Test ADORe simulation

If the following lines are executed, a plotlab figure should display a roadmap loaded from an exemplary OpenDrive file.

Start plotlab server:

~~~bash
cd ~/catkin_ws/src/adore/plotlab/server
./build.sh
./start.sh
~~~

Start launchfile:

~~~bash
source ~/catkin_ws/install/setup.bash
cd ~/catkin_ws/src/adore/adore_if_ros_demos
roslaunch demo001_loadmap.launch
~~~

For further information and demos see [adore_if_ros_demos](../adore_if_ros_demos/README.md).
