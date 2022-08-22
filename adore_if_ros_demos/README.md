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
*   Daniel Heß - initial API and implementation
********************************************************************************
-->
# ADORe ROS Demos
In the following, several demonstrations are given to showcase technical solutions on reduced examples.
Their goal is to serve as venture points for setting up simulation experiments with ADORe automated vehicles.
To run the demo scenarios, build the command-line-interface container and start it:
~~~bash
make build_adore-cli
make adore-cli
~~~
Then navigate to the catkin workspace demo directory in the adore-cli container, export your display variable and launch the demo:
~~~bash
cd catkin_workspace/src/adore_if_ros_demos
export DISPLAY=:0
roslaunch demo001_loadmap.launch
~~~

 
# Load Open Drive tracks
- file: [demo001_loadmap.launch](demo001_loadmap.launch) 
- The ROS node "adore_mapprovider_node" is responsible for loading tracks, aka. roadmaps.
- Track files are specified by the local ROS parameter "PARAM/track". ("local" as in "can be defined separately for each vehicle/agent in their respective namespace")
- Multiple tracks may be loaded: File names have to be separated by ";" in the "PARAM/track" string. 
- Loading multiple tracks results in a conjunction of elements from all tracks. If road elements of different tracks are geometrically adjacent, the automation system will be able to transition between them. 
- An offset may be specified for OpenDrive maps:
	- Append ",transform" to an OpenDrive file name specified in "PARAM/track"
	- Define west="x" and south="y" (lower left corner) in the header element of the OpenDrive xml file
	- All elements of the file will then be shifted by (x,y)
- The "adore_mapprovider_node" dissects the roadmap representation into lane- or road-border segments, of type "adore::env::borderbased::Border" 
- Each "Border" keeps track of its marking and of the lane type between itself and its left neighbor
- Typically an automated vehicle is allowed to drive between two Borders, which enclose a lane of type "Driving"
- To prevent the automation system's maneuver planner from having to deal with the complete map at once, the mapprovider will send "Borders" in the vicinity of the vehicle via the rostopic "ENV/Border". 
	- See also: The ros topic and message type documentation [adore_if_ros_msg/Readme.md](../adore_if_ros_msg/README.md)
	- The distance at which "Border" objects become visible is controlled by the parameter "PARAMS/map_provider/r"
- If the parameter "PARAMS/map_provider/activate_plotting" is set to "true", the overall map will be displayed in figure 1 

Overview plot of tracks loaded by demo001:
![Overview plot of tracks loaded by demo001](https://github.com/DLR-TS/adore_support/blob/master/demos/demo001.png)

# Computation of a navigation function
- file: [demo002_navigatemap.launch](demo002_navigatemap.launch)
- The ROS node "adore_navigation_node" helps an ADORe automated vehicle to navigate to its destination.
- A destination for the automated vehicle may be set, by sending a "NavigationGoal" message on the topic "ENV/NavigationGoal"
- Similar to the "adore_mapprovider_node", tracks specified in "PARAMS/track" are loaded into memory
- When such a "NavigationGoal" message is received, the distance along the road network towards the destination is computed for each "Border" object in memory
- Based on the same ideas of locality as the "adore_mapprovider_node", the "adore_navigation_node" sends distance values for each "Border" object in the vicinity of the vehicle on the ROS topic "ENV/NavigationData".
- If the parameter "PARAMS/navigation/active_plotting_global" is set to true, the navigation function will be shown color-coded for the overall map in figure 1. (Red is far, and green is near. Please note: In the example below, non-drivable sidewalks are color-coded red at "infinite" cost.)

Color-coded navigation function plotted by demo002:
![Color-coded navigation function plotted by demo002](https://github.com/DLR-TS/adore_support/blob/master/demos/demo002.png)

# Simulation of Automated Lane Following
- file: [demo003_lanefollowing.launch](demo003_lanefollowing.launch)
- Demo 003 shows how to simulated an automated vehicle with a minimal set of components. Several nodes are introduced in addition to the "adore_mapprovider_node" described above.
- To create a dynamic simulation, the ROS node "adore_timer_node" publishes a globally synchronous time signal on the topic "/SIM/utc". The process is started in a shell, in order to accept user input: By pressing enter, the simulation can be paused and resumed.
- The namespace "SIM" is here reserved for simulation information, e.g. global, synchronous, precise states accessible to simulation modules. 
- Topics under "SIM" can generally only be accessed by simulation nodes and never by "production" nodes. Simulation nodes, such as sensor models, vehicle models, network models etc. relay the information to sensor- and environment information topics of a specific agent/vehicle.
- In its most basic form, the "adore_vehiclemodel_node" serves both as a vehicle model and an idealized vehicle state sensor: It receives the time signal "/SIM/utc" and the vehicle control inputs "FUN/MotionCommand/steeringAngle" and "FUN/MotionCommand/acceleration" and integrates the vehicle state up to the most recent time value. This vehicle state is published on the topic "/_agentnamespace_/odom". (Note: In order to insert an error model for the state measurement, the output topic of "adore_vehicle_model" could be re-routed to an error-model node by declarations in the launch file.)
    - In order to set initial conditions for simulations, the state of the vehicle can be reset by sending messages on the topics "SIM/ResetVehiclePose" and "SIM/ResetVehicleTwist".
    - The state of the vehicle is always maintained for the center of the rear axle.
    - The "adore_vehiclemodel_node" also publishes its state information on the simulation topic "/SIM/traffic". In following exapmles, the topic is used to exchange state information between multiple, simulated agents/vehicles. A parameter "_agentnamespace_/simulationID" is introduced. This unique id is attached to the state information published on "/SIM/traffic", to discriminate between different agents during simulation.
- The "production" part of the automated lane following system consists of two control nodes in this example: A feed-forward motion planner running at 10 Hz and a stabilizing feed-back controller running at 100Hz.
    - The motion planner "adore_lfbehavior_node" generates trajectories for lane-following only. It receives "road marking information" on the topic "ENV/Border" from the "adore_mapprovider_node" and ego state information on the "odom" topic. It computes a reference trajectory consisting of desired vehicle states and feed-forward control inputs. (Desired states are given for the center of rear axle). The reference trajectories are published on the topic "FUN/SetPointRequest".
    - The feed-back controller receives the reference trajectories and the current vehicle state and computes and publishes a control input signal ("FUN/MotionCommand/steeringAngle" and "FUN/MotionCommand/acceleration") with the goal of minimizing the distance between actual vehicle state and current reference vehicle state.
- The process "adore_borderbird_node" plots a simplistic representation of the local scene from a "birds-eye" perspective: The local "road marking information", the reference trajectory (displayed in red) and the current vehicle state (blue box with triangle).

Local scene for lane following automation in demo003:
![Local scene for lane following automation in demo003](https://github.com/DLR-TS/adore_support/blob/master/demos/demo003.png)

# Lane following with three automated vehicles
- file: [demo004_follow_vehicle_mult.launch](demo004_follow_vehicle_mult.launch)
- In order to simulate multiple vehicles, multiple instances of "adore_vehiclemodel_node", "adore_mapprovider_node",  "adore_feedbackcontroller_node" and "adore_lfbehavior_node" have to be started.
    - Note: Each set of nodes is started in a separate namespace, e.g. "/vehicle0", "/vehicle1" and "/vehicle2"
- A simple sensor model is introduced, which allows a vehicle to detect positions of other vehicles: 
    - For each vehicle, an "adore_objectdetectionmodel_node" reads state information of traffic participants from the topic "/SIM/traffic" and "/SIM/traffic/agg". This information is published on the vehicle internal topic "/_agentnamespace_/traffic". 
    - The topic "/SIM/traffic" publishes individual state updates. On the topic "/SIM/traffic/agg", aggregate information in the form of multiple vehicle states per message is published. 
    - The "sensor modeling" part of the simple node is actually a range limit for detections.
    - Detections of other traffic participants are colored black in the figure 2 plot.
- Each vehicle can be parameterized independently in its namespace: The global speed limit of vehicle2, "/vehicle2/PARAMS/tactical_planner/global_speed_limit", is set to a small value in the launch file. This will result in vehicle 0 and 1 catching up  and following.
    - Note: The parameter can be manipulated during simulation with the shell command "rosparam set /vehicle2/PARAMS/tactical_planner/global_speed_limit 10". Of course, other parameters can also be changed during simulation.

Two fast vehicles following a slow vehicle in demo004:
![Two fast vehicles following a slow vehicle in demo004](https://github.com/DLR-TS/adore_support/blob/master/demos/demo004.png)

# Lane following with ADORe automated vehicle and multiple SUMO vehicles
- file: [demo005_follow_vehicle_sumo.launch](demo005_follow_vehicle_sumo.launch) 
- [SUMO](http://eclipse.org/sumo)  is a free and open traffic simulation toolsuite. It is used in this example to represent manually driven vehicles in the vicinity of the ADORe automated vehicle.
- SUMO has to be downloaded and installed. These steps will be executed by the cmake script of the SUMO-ROS interface "sumo_if_ros".  So before launching the demonstration, make sure to execute:
~~~bash
cd ~/catkin_ws/src/adore
catkin build sumo_if_ros
export SUMO_HOME=~/catkin_ws/src/adore/sumo
~~~
- The demo005 launch file starts a SUMO simulation. Parameters include SUMO simulation setup, (here: demo005.sumocfg), as well as a simulation step size.
- The node "sumotraffic2ros_node" uses SUMO's [TraCI](https://sumo.dlr.de/docs/TraCI.html) interface to synchronize simulation time and to exchange traffic participant state information.
    - On changes of the time signal "/SIM/utc" the SUMO simulation is advanced.
    - Traffic information from the topic "/SIM/traffic" is pusblised to SUMO, so that SUMO vehicles may react to ADORe vehicles.
    - Traffic information from SUMO is published on the "/SIM/traffic/agg".
- In this example, a single ADORe vehicle instance is created in addition to the SUMO traffic.
- NOTE: The timer is started in "paused" state, to allow startup of all process before simulation begins. Visual output will only be generated as soon as time advances, so make sure to "unpause" the timer node.

Several SUMO vehicles and one ADORe automated vehicle in demo005:
![Several SUMO vehicles and one ADORe automated vehicle in demo005](https://github.com/DLR-TS/adore_support/blob/master/demos/demo005.png)

# Lane following and navigation to a goal location
- file: [demo006_lanefollowing_navigation.launch](demo006_lanefollowing_navigation.launch)
- Demo006 showcases a vehicle automatically driving to a given goal position. The setup is similar to demo003, yet a "adore_navigation_node" is additionally started.
- The launch file sets an initial NavigationGoal. During simulation, the goal point can be changed by publishing a ROS message. For example with the following shell command:
~~~bash
rostopic pub /vehicle0/ENV/NavigationGoal adore_if_ros_msg/NavigationGoal '{target: {x: 360, y: 162, z: 0}}'
~~~ 
- As a reaction, the navigation function should be re-computed and the vehicle should change its course to drive to the new location.
- Note: Currently, the vehicle stops at the end of a "Border", which has minimum distance to the navigation goal. 
- NOTE: The timer is started in "paused" state, to allow startup of all process before simulation begins. Visual output will only be generated as soon as time advances, so make sure to "unpause" the timer node.

Vehicle turning at intersection and stopping at goal in demo006:
![Turning](https://github.com/DLR-TS/adore_support/blob/master/demos/demo006_1.png)![Stopping](https://github.com/DLR-TS/adore_support/blob/master/demos/demo006_2.png)

# Prediction
- file: [demo007b_precedence_no_rules.launch](demo007b_precedence_no_rules.launch)
- In demo007b two ADORe automated vehicles approach an unsignalized intersection and avoid a collision by predicting possible paths of the vehicle.
- Vehicle0 approaches from west and turns north, vehicle1 crosses the intersection straight from south to north.
- The [adore_prediction_provider](../adore_if_ros/src/adore_prediction_provider.cpp) node is executed by each vehicle and computes for each detected traffic participant a set of locations that might be covered by the given traffic participant in the future.
- Predictions are published on the topic "ENV/Prediction"
- A collision detection module (for example in adore_lfbehavior_node) receives the predictions and intersects possible plans of the ego vehicle with possible positions of other traffic participants 
- In case of a conflict, a planned trajectory may not be executed and thus the vehicle decelerates
- In the given demonstration scenario, the vehicles initially both react to each other and decelerate
- As vehicle0 is approaching a turn, it is initially slower and reaches standstill before vehicle1
- As soon as vehicle0 is stopped, vehicle1's path is clear and it may continue across the intersection
- Obviously, such a coordination at an intersection is suboptimal and therefore the next section outlines improved behavior through prediction in combination with static precedence rules
![vehicle0 and vehicle1 predict each other's path](https://github.com/DLR-TS/adore_support/blob/master/demos/demo007b.png)

# Prediction and precedence rules
- file: [demo007a_precedence_right.launch](demo007a_precedence_right.launch)
- In demo007a two ADORe automated vehicles approach an unsignalized intersection and solve the impeding conflict according to a static set of precedence rules
- The setup is similar to demo007b, with the addition of static precedence rules defined for the intersection
- The static precedence rule set is loaded from a [file](tracks/basic_test_track_precedence_v2.txt) by adore_mapprovider_node 
- The file path is given by the parameter "PARAMS/precedence"
- adore_mapprovider_node disseminates all static precedence rules near the vehicle via the topic "ENV/Precedence"
- A precedence file specifies a list of connection pairs (four coordinates) "X0,Y0,Z0;X1,Y1,Z1 > X2,Y2,Z2;X3,Y3,Z3"
- A connection pair "Ca > Cb", Ca="c0;c1"="X0,Y0,Z0;X1,Y1,Z1", Cb="c2;c3"="X2,Y2,Z2;X3,Y3,Z3" defines that a connection passing along c0 and c1 has precedence over a connection passing along c2 and c3.
- The [adore_prediction_filter](../adore_if_ros/src/adore_prediction_filter.cpp) node receives the static precedence rules as well as the predictions of adore_prediction_provider
- The prediction filter selects the high priority connections in the path of the ego vehicle and removes all branches from the prediction tree, which lie on corresponding low priority connections
- The modified prediciton set is published again and allows to apply only those predictions to collision detection, which do not violate the precedence of the ego vehicle
- In demo007a vehicle1 has precedence over vehicle0 and by considering the filtered predictions of vehicle0 may cross the intersection unhindered.

![vehicle1 predicts vehicle0 to yield at the intersection](https://github.com/DLR-TS/adore_support/blob/master/demos/demo007a.png)

- file: [demo007c_precedence_left.launch](demo007c_precedence_left.launch)
- Demo007c is similar to demo007a, but the precedence [file](tracks/basic_test_track_precedence_v3.txt) is modified to give precedence to the connection leading from west to north
- Accordingly, vehicle0 crossses the intersection before vehicle1.

![vehicle0 predicts vehicle1 to yield at the intersection](https://github.com/DLR-TS/adore_support/blob/master/demos/demo007c.png)

# Straight-line Predictions
- file: [demo007d_sl45_right_turn_pedestrian.launch](demo007d_sl45_right_turn_pedestrian.launch)
- In demo007d an ADORe vehicle approaches a right turn at an intersection. A stationary object (pedestrian) is detected. As the vehicle starts to enter the turn, the pedestrian starts crossing the road.
- As the pedestrian is not using a known crosswalk, the prediction module uses only the pedestrian's linear motion vector to extrapolate future positions.
- In the current version only the two prediction modes 'along lane' (demo007a-c) and 'straight' are available (demo007d). The successfule lane matching of objects determines, which mode is used.
- Demo007d gives an example for triggering events in a simulation: Using the transform node of the topic_tools ROS package, the velocity of the pedestrian is set according to the position of vehicle0:
```bash
rosrun topic_tools transform /vehicle0/odom /pedestrian0/SIM/ResetVehicleTwist geometry_msgs/Twist 'geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2 if m.pose.pose.position.x>662 else 0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3( x= 0.0, y= 0.0, z= 0.0))' --import geometry_msgs std_msgs
``` 
![vehicle0 approaching intersection](https://github.com/DLR-TS/adore_support/blob/master/demos/demo007d01.png)
![pedestrian starts walking](https://github.com/DLR-TS/adore_support/blob/master/demos/demo007d02.png)
![vehicle0 stopping for pedestrian](https://github.com/DLR-TS/adore_support/blob/master/demos/demo007d03.png)
![vehicle0 continuing after pedestrian](https://github.com/DLR-TS/adore_support/blob/master/demos/demo007d04.png)

# Traffic Lights
- file: [demo008_sumo_trafficlights.launch](demo008_sumo_trafficlights.launch)
- Demo008 showcases traffic lights detected via MAPEM and SPATEM V2X radio messages
- MAPEM messages are sent by roadside units at intersections to inform about the intersection geometry and topology.
- SPATEM messages are sent by roadside units at intersections to inform about current (and optionally future) traffic light signal phases.
- A co-simulation with SUMO is run. The SUMO scenario does not specify any traffic, so SUMO manages only the traffic lights in the given scenario.
- Besides publishing to the topic /SIM/traffic, the node sumo_if_ros/sumotraffic2ros_node publishes to the topics /SIM/v2x/MAPEM and /SIM/v2x/SPATEM.
- The messages published to /SIM/v2x are wrappers, which contain ROS message equivalents of ETSI ITSG5 messages as payload, as well as simulation meta information, such as sending position and signal strength. 
- A node of type v2xsim/channel_sim_node is run for vehicle0. The channel_sim_node decides based on ego position, sender position and signal strength, whether a message on the topic /SIM/v2x is received. Received messages are copied from the simulation topic /SIM/v2x to the vehicle specific topic /vehicle0/v2x/incoming. 
- The package adore_if_v2x is the interface to ITSG5 ROS message equivalents. The node adore_if_v2x/v2x_trafficlights_node receives MAPEM and SPATEM messages from /vehicle0/v2x/incoming and converts these into ADORe internal data formats published on /vehicle0/ENV/tcd.
- In case of a different sensor type identifying signal phases (most likely camera-based), the information would also be published to /vehicle0/ENV/tcd.
- The topic /vehicle0/ENV/tcd transmits signal states for all known connections, agnostic to their relevance to the ego vehicle. Trajectory planning processes, such as adore_lfbehavior_node match the data to lane-following and lane-change views to determine where to stop the ego vehicle, while the adore_prediction_filter applies the signal phases to the prediction of other traffic participants near ego.
- As shown in the image below, current traffic light phases are visualized as icons as well as connection vectors crossing the intersection.
![vehicle0 approaching a redlight](https://github.com/DLR-TS/adore_support/blob/master/demos/demo008.png)

# Lane Changes
- file: [demo009_lanechange_tostmannplatz.launch](demo009_lanechange_tostmannplatz.launch)
- In demo009 a vehicle has to navigate through a complex intersection. To reach the goal point in the north-west intersection arm from the starting position in the south, multiple lane changes are required.
- The extended vehicle [configuration](demo_vehicle_cfg03.launch) for this scenario starts a decision making module "adore_tactical_planner_node", a "adore_trajectory_planner_lf_node" for lane following as well as two instances each of "adore_trajectory_planner_lc_node" and "adore_trajectory_planner_lm_node" for planning lane changes to the left and to the right. 
- The lane change planner "adore_trajectory_planner_lc_node" computes trajectories for preparation and initiation of lane changes, while the lane merge planner "adore_trajectory_planner_lm_node" plans trajectories for the final part of the lane change, merging into the target lane.
- The decision making module sends PlanningRequest messages and receives PlanningResult messages from active trajectory planners. It selects "the best" trajectory and sends it to the trajectory tracking controller to be executed. In order to allow comparison of trajectories, each planner provides a set of different objective values for a valid trajectory. The "adore_tactical_planner_node" compares trajectories purely on their validity and the objective value of "MinimumNavigationCostOnLane".
- Indicators are currently not visualized, but their activation can be examined on the topic /vehicle0/FUN/IndicatorCommand. Each PlanningResult specifies which indicator has to be activated. The activation command is generated by the tactical planner node based on the selected trajectory.
![vehicle0 changing lanes](https://github.com/DLR-TS/adore_support/blob/master/demos/demo009.png)

# Background satellite images
- file: [demo010_sat_images.launch](demo010_sat_images.launch)
- Using satellite images as a background for the lane geometry can help to check for mapping errors.
- Demo010 loads satellite images from a publicly available [geo-server](https://www.wms.nrw.de/).
- A lane model for a small section of the city of Duesseldorf, Germany [(51.21564822166936, 6.775329829362242)](https://www.openstreetmap.org/search?query=d%C3%BCsseldorf%2C%20herzogstr#map=19/51.21506/6.77778) is shown on top of the satellite images.
- The ROS node adore_if_ros/plot_satimages_node requests plotlabserver to plot url-based images near the position of the ego vehicle.
- The plotlabserver will download and cache the images. (Plotting may stall when images are displayed for the first time.)
- Parameters "plotoptions/tiles/base_url" and "plotoptions/tiles/width_meters" control geo-server url, image resolution and image size in m:
```xml
    <param name="plotoptions/tiles/base_url" value="https://www.wms.nrw.de/geobasis/wms_nw_dop?language=ger&#038;SERVICE=WMS&#038;REQUEST=GetMap&#038;VERSION=1.3.0&#038;layers=nw_dop_rgb&#038;styles=&#038;width=400&#038;height=400&#038;CRS=EPSG:25832&#038;FORMAT=image/jpeg&#038;bbox=" type="str" /> 
    <param name="plotoptions/tiles/width_meters" value="100" type="double" /> 
```
![duesseldorf scenario satellite images](https://github.com/DLR-TS/adore_support/blob/master/demos/demo010.2.png)
![duesseldorf scenario satellite images, zoom](https://github.com/DLR-TS/adore_support/blob/master/demos/demo010.png)

# Faster (slower) than real-time and background simulation
- file: [demo011_scheduler.launch](demo011_scheduler.launch)
- The scenario of this example is equivalent to [demo006_lanefollowing_navigation.launch](demo006_lanefollowing_navigation.launch)
- The approach to simulating the scenario has been adapted for "headless" applications, such as batch simulations for validation and machine learning. 
- The previously used simple timer (which allowed to pause and unpause simulation) has been replaced with a scheduler: The adore_scheduler_node communicates with all time-critical processes and waits for the finalization of a given time slice, before advancing the simulated time. The scheduler allows the simulation to run faster than real-time, in case sufficient computational ressources are available. (This should typically be the case for the given scenario.) In case the scenario is complicated and the computational resources are limited, the scheduler allows to simulate slower than real-time, with all time-critical processes executing as if sufficient resources permitted finalization of computations in time.
- The demo's run time on an Intel Core i7 notebook is about 17s (computation time) versus 120s of simulated time. (Disclaimer: Of course the demo case is very simplified and the runtime factor for a simulation with other traffic participants and multiple trajectory planners for ego might be considerably worse. The presented setup merely serves to showcase that a runtime factor differing from 1 can be achieved.)
- The simulation automatically terminates, if the vehicle is near the goal location or if 130s of simulation time have been surpassed. 
- The adore_monitor0_node publishes a proposition on the topic "/vehicle0/ENV/propositions" with term="NEAR_GOAL" and value true or false.
- A topic_tools "transform" node has been added to the launch file, which filters the propositions topic for NEAR_GOAL=true and publishes to /SIM/terminate.
- The adore_ci_terminator_node listens on /SIM/utc and /SIM/terminate topics and will shut down the simulation accoordingly (the flag required="true" has been added to the node tag and the node self-terminates to shutdown simulation)
- An argument has been added to the demo launch file, which allows to run the simulation with or without graphical output:
```bash
roslaunch demo011_scheduler.launch headless:=true
```
![headless simulation](https://github.com/DLR-TS/adore_support/blob/master/demos/demo011xterm.png)

# Speed Limit adherence

- file: [demo012_lanefollowing_w_speedlimit.launch](demo012_lanefollowing_w_speedlimit.launch)
- This is an adaption of [demo003_lanefollowing.launch](demo003_lanefollowing.launch) which adds the speedlimit_provider node. The ego will slow down a bit near the start and again after about 300m.
- The file [speedlimit.txt](speedlimit.txt) is used as a source for speed limits in a simple csv. Each line has 5 values, the first is the speed limit in m/s and the next two pairs represent the start x/y and stop x/y.
- The speedlimit_provider node reads this file in, given as a parameter, and will publish each set of data if the ego is close enough. This information is then used in the computation of the laneview provider. There a speedlimit function is derived for each lane, from the global speed limit and each set of speed limit data where both start and stop points are within the lane.
- The demo csv file contains some off lane speed limits to show that these are ignored.


# Simulation of localization errors and odometry drift
Automated Vehicles often determine their position via (differential) GPS, comparison of LIDAR measurements with an a-priori map, other optical features with an a-priori map or a combination thereof. New observations may lead to a correction of the belief where the AV is located. Feed-back controllers with a high gain should not be directly exposed to such jumps in the belief state, as that could lead to instability or at least uncomfortable control actions. Wheel-speed/rotation measurements, inertial measurement units and optical-differential measurements are often summerized as odometry based information: A common characteristic is that an odometry-based belief state is jump free but drifts (slowly diverges from ground truth with time). Its drift error rules out comparison of the odometry belief against globally referenced information, as is the case for trajectory planners and HD-maps in ADORe.
By employing localization-based states for predictive feed-forward computation and odometry-based states for feedback, the problems of both signals can be compensated. Such a technique appears to be state-of-the-art and is for example utilized by the ROS [navigation stack](http://wiki.ros.org/navigation/Tutorials/RobotSetup).

The approach is demonstrated for ADORe as follows: Error models for localization and odometry are used in the modified configuration-file [demo_vehicle_cfg01b.launch](demo_vehicle_cfg01b.launch). Both make use of the ground truth (luckily avaliable in simulation), which is supplied by the vehicle model on the topic `/vehicle0/SIM/state` and supply their error-augmented outputs via `/vehicle0/localization` and `/vehicle0/odom`. 
- file: [demo013_localization_errors.launch](demo013_localization_errors.launch)
- The vehicle model is started with the argument `external_ego_measurement_models:=true` to suppress error-free outputs. In this case the vehicle model only supplies the topic `/vehicle0/SIM/state`.
- The additional process adore_odometrymodel_node supplies the odometry estimate
- The additional process adore_localizationmodel_node supplies the localization estimate

The following image shows a situation from demo013, with ground-truth vehicle state given in red (also vehicle image) and localization belief state in blue. The setup allows the trajectory planner to assess whether a localization update necessitates sharp control action or whether the offset may be corrected over a prolonged period of time. The trajectory planner selects the initial state of the new trajectory at the old reference state in odometry coordinates and calculates where the initial state is located under consideration of the new localization update. Thus guaranteeing continuity in odometry coordinates while also allowing to react to changes in the localization frame.

![Odometry based control and localization errors](https://github.com/DLR-TS/adore_support/blob/master/demos/demo013.png)

Note: 
- The operation of the vehicle in error-free conditions may be simulated by supplying the argument `external_ego_measurement_models:=false` (or no argument) to the vehicle model and by starting neither adore_odometrymodel_node nor adore_localizationmodel_node.
- The vehicle may be operated on purely localization information by supplying the localization state also on the topic `/vehicle0/odom` instead of the odometry state.

# CARLA Coupling
- file: [demo014_adore_if_carla.launch](demo014_adore_if_carla.launch)
- Note: adore_if_carla is experimental.
The demo shows the the coupling of ADORe and CARLA.
In order to run this demo, the adore_if_carla package needs to be built. Further instructions on that can be find in the [README](https://github.com/DLR-TS/adore_if_carla/blob/main/README.md).
To start the demo, the following steps need to be performed:
1. build adore_if_carla
2. start CARLA by running [adore_if_carla/run_1_carla.sh](https://github.com/DLR-TS/adore_if_carla/blob/main/run_1_carla.sh)
3. start the CARLA-ros-bridge by running [adore_if_carla/run_2_carla_ros_bridge.sh](https://github.com/DLR-TS/adore_if_carla/blob/main/run_2_carla_ros_bridge.sh)
4. start the adore_if_carla core-nodes by running [adore_if_carla/run_3_adore_if_carla.sh](https://github.com/DLR-TS/adore_if_carla/blob/main/run_3_adore_if_carla.sh)
5. start the [demo014_adore_if_carla.launch](demo014_adore_if_carla.launch) of adore_if_ros_demos

![ADORe-CARLA-interface](https://github.com/DLR-TS/adore_support/blob/master/demos/demo014.png)