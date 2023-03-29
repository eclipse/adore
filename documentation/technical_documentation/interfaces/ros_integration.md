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
* Daniel Heß
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
The header ```adore_if_ros_scheduling/baseapp.h``` defines a base class, which is used in adore_if_ros control process nodes in order to execute and schedule the control process defined in ```libadore/apps```. 

The most important ros nodes provided by the adore_if_ros package are:

| purpopse                  | ROS node                  | description                                                      |
| ------------------------- | ------------------------- | ---------------------------------------------------------------- |
| planning&control | adore_checkpoint_controller_node | monitors a predefined list of manual checkpoints and enforces driver interaction before vehicle may continue over a given checkpoint|
| planning&control | adore_feedback_controller_node | tracks the current trajectory and outputs steering and acceleration actuator values|
| planning&control | adore_lfbehavior_node | lane following only tactical layer |
| planning&control | adore_lvprovider_node | computes and publishes lane following and lane change views |
| planning&control | adore_mission_controller_node | vehicle's top level state automaton responsible for switching between driving/parking and next goal location |
| planning&control | adore_monitor0_node | computes some zero order logic propositions about vehicle state, e.g. IN_COLLISION|
| planning&control | adore_navigation_node | publishes navigation information (cost to go) for lanes near vehicle |
| planning&control | adore_tactical_planner_node | requests plans from trajectory planners and selects one of the results for execution|
| planning&control | adore_trajectory_planner_lc_node | plans lane change trajectories |
| planning&control | adore_trajectory_planner_lf_node | plans lane following trajectories |
| planning&control | adore_trajectory_planner_lm_node | plans lane merge trajectories (finalization of lane change maneuver) |
| environment model | adore_mapprovider_node | publishes static lane and precedence information near vehicle |
| environment model | adore_prediction_filter | filters traffic participant motion predictions for relevance to the ego vehicle|
| environment model | adore_prediction_provider | comutes motion predictions in form of occupied space time for traffic participants detected near ego vehicle|
| environment model | area_of_effect_provider | computes area that is accessible to ego vehicle |
| environment model | speedlimit_provider | publishes speedlimit information near ego vehicle |
| environment_model | adore_crosstraffic_provider | Monitors prediction of other traffic participants and tests for intersection of cross-traffic with lane following view. In case of crosstraffic intersecting the lane following view, the nearest intersection with a feasible stopping position is published on the topic ENV/ConflictSet. The node may be extended in the future to publish also all intersection points after the first potential stopping position. |
| simulation | adore_ci_terminator_node | terminates an automatic simulation |
| simulation | adore_objectdetectionmodel_node| simulation module, which transfers information about objects near vehicle from simulation topic to ego vehicle topic|
| simulation | adore_scheduler_node| controls simulation time and advances simulation time according to slowest process|
| simulation | adore_timer_node | constrols simulation time and advances simulation time according to system time (when not paused) |
| simulation | adore_vehiclemodel_node | controls vehicle state in simulation according to scenario initialization and control inputs|
| simulation | adore_odometrymodel_node | simulates typical errors for odometry based state estimates |
| simulation | adore_localizationmodel_node | simulates typical errors for GPS-localization based state estimates |

### Communication Patterns
The header ```adore/mad/com_patterns.h``` in libadore defines different abstract communication pattners, which are realized in ```adore_if_ros/ros_com_patterns.h``` with the help of ROS's publisher/subscriber principle.

### Data Exchange and Topic Definition
In order to define the availability of different data sources and sinks in a system independent way, libadore makes use of an abstract factory pattern. Three abstract factories are defined in libadore for the data exchange of the environment model ```adore/env/afactory.h```, the control functions ```adore/fun/afactory.h``` and simulation data (ground truth) ```adore/sim/afactory.h```.

The concrete factories in adore_if_ros, ```envfactory.h```, ```funfactory.h``` and ```simfactory.h``` define the ROS topics on which data is exchanged, the message format in which data is exchanged, the transformation method to translate between ROS messages and c++ objects and create ROS Publisher and Subscriber instances to facilitate the transport. The concrete factories reference transformation classes defined in ```adore_if_ros/conversions```

### Parameters
The abstract factory defined in ```adore/params/afactory.h``` is realized by ```adore_if_ros/paramsfactory.h```: The concrete instances define parameter paths and names used in the ROS environment and query the parameter server.

### ROS Namespaces
ADORe discriminates namespaces of multiple vehicles. Nodes, communication topics and parameters of a given vehicle are grouped in a collective namespace, for example ```/vehicle0/```. This allows to simulate multiple autonomous vehicles in the same namespace. An arbitrary namespace can be given to each vehicle, but it must be distinct. The namespace ```/SIM``` is reserved for simulation data, meaning the ground truth of the simulation. Accordingly the topic ```/SIM/traffic``` will contain all participant's error-free states, while ```/vehicle0/traffic``` contains observations of vehicle0 about other traffic participants. Similarly ``` /SIM/v2x``` contains all transmitted V2X messages and ```/vehicle0/v2x/incoming``` contains messages received by vehicle0.

