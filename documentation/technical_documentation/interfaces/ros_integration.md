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
********************************************************************************
-->
## ADORe interfacing with ROS
ADORe core functionality is provided by the system-independent c++ library libadore. 
The module adore_if_ros integrates library functionality into a ROS environment: 
Control processes in ```libadore/apps``` are wrapped by corresponding ROS nodes.
The communication between different ROS nodes and their corresponding control processes is transported via messages defined in adore_if_ros_msg or in the ROS standard message packages. 
The module adore_if_ros defines transformations between libadore internal c++ classes and ROS messages.
The ROS parameter server provides values for the libadore internal parameters.
``` ```
### Control Processes and ROS Nodes 
The header ```adore_if_ros/baseapp.h``` defines a base class, which is used in adore_if_ros control process nodes in order to execute and schedule the control process defined in ```libadore/apps```. 

### Communication Patterns
The header ```adore/mad/com_patterns.h``` in libadore defines different abstract communication pattners, which are realized in ```adore_if_ros/ros_com_patterns.h``` with the help of ROS's publisher/subscriber principle.

### Data Exchange and Topic Definition
In order to define the availability of different data sources and sinks in a system independent way, libadore makes use of an abstract factory pattern. Three abstract factories are defined in libadore for the data exchange of the environment model ```adore/env/afactory.h```, the control functions ```adore/fun/afactory.h``` and simulation data (ground truth) ```adore/sim/afactory.h```.

The concrete factories in adore_if_ros, ```envfactory.h```, ```funfactory.h``` and ```simfactory.h``` define the ROS topics on which data is exchanged, the message format in which data is exchanged, the transformation method to translate between ROS messages and c++ objects and create ROS Publisher and Subscriber instances to facilitate the transport. The concrete factories reference transformation classes defined in ```adore_if_ros/conversions```

### Parameters
The abstract factory defined in ```adore/params/afactory.h``` is realized by ```adore_if_ros/paramsfactory.h```: The concrete instances define parameter paths and names used in the ROS environment and query the parameter server.

