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
 export SUMO_HOME="/home/akoerner/repos/csa/github.com-DLR-TS/test1/adore/sumo"

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
cd 
./bin/sumo --remote-port 1337 -c tests/complex/tutorial/quickstart/data/quickstart.sumocfg --step-length 0.01
~~~

The TraCI client can be connected as follows:
~~~c++
TraCIAPI client;
client.connect("localhost", 1337);
~~~


## Generate a network file
One option is to convert OpenDrive files directly:

See sumo doc on [netgenerate](https://sumo.dlr.de/docs/NETGENERATE.html):
~~~bash
netconvert --opendrive myOpenDriveNetwork.xodr -o mySUMOnetwork.net.xml --offset.disable-normalization 
~~~

Another option is to convert maps, which are readable by adore, into SUMO PlainXML format [SUMO PlainXML specification](https://sumo.dlr.de/docs/Networks/PlainXML.html): ADORe provides a ```plainxmlexporter``` binary, available from the selected binary folder, e.g. ```install/lib/adore_if_ros```.
Provide one or more map file paths as argument to binary in order to create PlainXML output ```output.nod.xml``` node defintions, ```output.edg.xml``` edge definitions and ```output.con.xml``` connection definitions.
Currently, xodr and r2s maps can be read and converted.
The parameters for plainxmlexporter are ```[plot] infile1[,transform] [infile2[,transform]] ... outfile```, where plot allows immediate output of loaded files to plotlab, each infile specifies path to a map file and outfile defines prefix of Plain-XML output files. 
Road2Simulation input files (file1.r2sl, file1.r2sr) have to be specified as (file1.r2s) in a single infile name.
The transform arguments allow to enable/disable application of transforms specified in the OpenDrive header.
The resulting PlainXML files can be further converted to a Net-XML format using sumo ```netconvert```.

Here is an example:
~~~bash
cd ~/catkin_ws

prefix=src/adore/adore_if_ros_demos/tracks/r2s/tost2dlr

install/lib/adore_if_ros/plainxmlexporter .r2s 

src/adore/sumo/bin/netconvert \
  --node-files=.nod.xml \
  --edge-files=.edg.xml \
  --connection-files=.con.xml \
  --output-file=.net.xml
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
