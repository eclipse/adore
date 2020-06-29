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
# Automatic SUMO integration
The cmake files of the sub-project sumo_if_ros are configured to automatically download and install SUMO in the folder catkin_ws/src/adore/sumo.
To trigger the automatic installation, build sumo_if_ros:
~~~bash
cd ~/catkin_ws/src/adore
catkin build sumo_if_ros
~~~
The ros node sumo_if_ros/sumotraffic2ros advances SUMO time steps according to the ADORe simulation time and exchanges data between SUMO and ROS.
See section adore_if_ros_demos for examples.

# Manual SUMO integration

The following steps can be found in the [SUMO documentation](https://sumo.dlr.de/docs/Installing/Linux_Build.html) and have been slightly modified to attribute for file paths.

Install a complete instance of sumo in the src folder:
~~~bash
 sudo apt-get install cmake python g++ libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev swig

 cd ~/catkin_ws/src/adore
 git clone --recursive https://github.com/eclipse/sumo
 export SUMO_HOME="$PWD/sumo"

 mkdir sumo/build/cmake-build && cd sumo/build/cmake-build
 cmake ../..
 make -j$(nproc)
~~~

The files for TraCI/c++ interfacing with sumo are located here:
~~~bash
sumo/src/utils/traci/TraCIAPI.h
sumo/src/utils/traci/TraCIAPI.cpp
sumo/src/foreign/tcpip/socket.h
sumo/src/foreign/tcpip/socket.cpp
sumo/src/foreign/tcpip/storage.h
sumo/src/foreign/tcpip/storage.cpp
sumo/src/traci-server/TraCIConstants.h
sumo/src/libsumo/TraCIDefs.h
~~~

If you start sumo with the following parameters:
~~~bash
cd ${SUMO_HOME}
./bin/sumo --remote-port 1337 -c tests/complex/tutorial/quickstart/data/quickstart.sumocfg --step-length 0.01
~~~

The TraCI client can be connected as follows:
~~~c++
TraCIAPI client;
client.connect("localhost", 1337);
~~~


## Generate a network file
See sumo doc on [netgenerate](https://sumo.dlr.de/docs/NETGENERATE.html):
~~~bash
netconvert --opendrive myOpenDriveNetwork.xodr -o mySUMOnetwork.net.xml --offset.disable-normalization 
~~~

## Define a route description
Define a file routes.xml [schema](https://sumo.dlr.de/xsd/routes_file.xsd), which contain vehicle and flow.
~~~xml
<routes>
    <vType id="traffic" accel="0.8" decel="4.5" sigma="0.5" length="5" maxSpeed="70"/>
    <flow id="id" type="traffic" begin="0" end="3600" probability="0.1" from="start" to="end" />
</routes>
~~~

## Define a re-router in additional file
Example for circular track can be found [here](https://sumo.dlr.de/docs/Tutorials/Driving_in_Circles.html)
~~~xml
<additionals>
    <rerouter id="rerouter_0" edges="-1.0.00">
        <interval end="1e9">
           <destProbReroute id="-1.157.08"/>
        </interval>
    </rerouter>
    <rerouter id="rerouter_1" edges="-1.157.08">
        <interval end="1e9">
           <destProbReroute id="-1.0.00"/>
        </interval>
    </rerouter>
</additionals>
~~~