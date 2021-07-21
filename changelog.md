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
********************************************************************************
-->
# Change Log

## 0.2 - 2021-07-16
* Migration to Ubuntu 20.04 and ROS noetic
* New support for Road2Simulation map format
* Several new modules have been added with this version, which are discussed in the following:

* adore_if_ros_demos: New application examples
  * demo007a_precedence_right.launch demonstrates defining and adhering to right of way with the help of road-based predictions
  * demo007b_precedence_no_rules.launch serves as comparison to 007a: Behavior with undefined right of way
  * demo007c_precedence_left.launch demonstrates definition of right of way opposite to case 007a
  * demo007d_sl45_right_turn_pedestrian.launch demonstrates non-road-based prediction of traffic participant and conflict resolution
  * demo008_sumo_trafficlights.launch demonstrates definition of scenarios with traffic lights controlled by sumo and simulation of MAPEM and SPATEM V2X messages, as well as reaction of vehicle to messages
  * demo009_lanechange_tostmannplatz.launch demonstrates lane change maneuver planning and maneuver selection by tactical planner on a realistic urban roadway
  * demo010_sat_images.launch demonstrates how to integrate satellite image data from wms geo service for visualization
  * demo011_scheduler.launch demonstrates faster/slower than real-time as well as headless simulation for "batch processing" of scenarios
* adore_if_ros: New vehicle control nodes
  * adore_checkpoint_controller_node - ROS node, which monitors a predefined list of checkpoints and enforces driver interaction before vehicle may continue over a given checkpoint
  * adore_lvprovider_node - ROS node, which computes adore::views (lane following and lane change) and provides these to other adore processes
  * adore_mission_controller_node - ROS node with vehicle's top level state automaton responsible for switching between driving/parking and next goal location
  * adore_monitor0_node - ROS node which computes some zero order logic propositions about vehicle state, e.g. IN_COLLISION
  * adore_prediction_filter - ROS node which filters predictions for relevance to the ego vehicle
  * adore_prediction_provider - ROS node which comutes motion predictions in form of occupied space time for traffic participants detected near ego vehicle
  * adore_tactical_planner_node - ROS node which requests plans from trajectory planners and selects one of the results for execution
  * adore_trajectory_planner_lc_node - ROS node which plans lane change trajectories
  * adore_trajectory_planner_lf_node - ROS node which plans lane following trajectories
  * adore_trajectory_planner_lm_node - ROS node which plans lane merge trajectories (finalization of lane change maneuver)
  * area_of_effect_provider - ROS node which computes area that is accessible to ego vehicle
  * plainxmlexporter - An offline, std cpp, standalone process, which converts from ADORe map formats to SUMO plainXML transfer format
* adore_if_ros: Changes to plotting tools (for plotlab)
  * adore_borderbird_node - deleted
  * plot_graph_node - ROS node for graph plotting of numeric ROS topics with plotlab
  * plot_area_off_effect - visualize output of area_of_effect_provider
  * plot_ego_node - visualize ego vehicle as simple line drawing or .png image
  * plot_lanes_node - visualize lanes near ego vehicle
  * plot_plan_swath_node - visualize space covered by ego when following planned trajectories
  * plot_predictions_node - visualize prediction of space covered by detected traffic participants
  * plot_satimages_node - displays sattelite images in background of top view plot; images are downloaded from specified geoserver
  * plot_traffic_node - visualize detected traffic participants as simple line drawings or .png images
  * plot_trajectory_node - visualize all currently available trajectories
  * plot_views_node - visualize lane-following and lane-change views
* adore_if_ros: Some auxiliary nodes for testing
  * test_control_dashboard - plots some values, which are insightful for debugging and controller tuning
  * test_control_error_node - allows to induce control errors in order to test driver reaction times
  * test_lc_trajectory_planner_node 
  * test_mrm_planner_node
  * test_setspeedlimit - allows to insert speed-limit changes at certain locations
  * test_straight_line_prediction_node
  * test_trajectory_planner_node
  * test_vectorsize.py - rostopic style command line tool: displays size of vectors contained in ROS messages
  * plotlab2navigationgoal.py - publishes navigation goal, if a coordinate is selected in plotlab (pressing 'l')
* adore_if_ros_msg (ROS messages for ADORe ROS interface)
  * AreaOfEffect.msg - determines area that could be affected by vehicle, allows to filter objects accordingly
  * CostPair.msg - pair of objective value name and objective value, used in PlanningResult.msg
  * LaneChangeGeometry.msg - encodes information for adore::view::ALaneChangeView
  * LaneFollowingGeometry.msg - encodes information for adore::view::ALane
  * LaneGeometry.msg - combines LaneChangeGeometry.msg and LaneFollowingGeometry.msg
  * LinearPiecewiseFunction*d.msg - linear, piecewise functions
  * MissionState.msg - state information for mission controller
  * OccupancyCylinder.msg - Space time volume for collision detection system
  * OccupancyCylinderPrediction.msg - A predicted sequence of OccupancyCylinders, can be associated with a tracked object and can form the branch of a prediction tree
  * OccupancyCylinderPredictionSet.msg - A set of OccupancyCylinderPredictions
  * PlanningRequest.msg - Used to request trajectory planners to start planning for a given initial state 
  * PlanningResult.msg - A trajectory planner's result, reply to PlanningRequest.msg
  * PlanningResultSet.msg - A set of PlanningResult.msg
  * PlotStart.msg - A message notifying plotter to start plotting numeric values of given ROS topics
  * TCDConnectionStateTrace.msg - The signal state over time for a singel connection path at an intersection

* v2x_if_ros (V2X ROS interface)
  * autogenerated ROS messages for ETSI ITSG5 standardized and test V2X radio messages
   * CAM v2 (standardized) - publish vehicle state
   * CPM v1_19 (standardized) - distributed awareness, publish vehicle observations
   * DSRC v2 (standardized) (MAPEM, SPATEM, SREM, SSEM) - intersection topology and traffic light signal phase timing, signal phase requests
   * MCM transaid - maneuver coordination message, publish vehicle intended behavior, request vehicle behavior adaptation
   * MCM dmove - extension of MCM transaid version: request vehicle to follow specific trajectory
   * ODDsinfo - publish operational domain information
   * STRP fau - space time reservation for vehicle motion coordination
  * v2x_sim: simulation of v2x radio message transmission
   * channel_sim_node - ROS node for handover between a simulated station and the simulated V2X radio channel

* adore_if_v2x (ADORe interface to ROS V2X messages)
  * mcm_to_prediction_node - ROS node for conversion of a received MCM V2X message into a vehicle prediction processable by ADORe vehicle
  * setpointrequest_to_mcm_node - ROS node for publishing the trajectory that will be executed by ADORe vehicle
  * v2x_trafficlights_node - ROS node for conversion of MAPEM and SPATEM V2X messages into signal state information for ADORe vehicle
* sumo_if_ros (eclipse SUMO interface to ROS)
  * sumotls2ros_node - ROS node, which extracts traffic light signal phases from SUMO via TraCI and publishes the information as MAPEM and SPATEM messages
* keepmoving_if_ros: Communication with REST server to aquire mission goals and provide information about ego vehicle. (preliminary devolpment stage, future changes are expected)
  * shuttle.py - monitors ego vehicle state and pushes info to REST server for fleet management
  * journey.py - requests state, route and target information from REST server

## 2020-09-29

* Updated lane view provider node, does not support lane following and lane change views

## 2020-08-10

* New machine learning functionality allows training of reinforcement learning agents.

## 0.1.1 - 2020-05-13

* Modifications of IP check
  * removed license information from dependencies.md files
  * removed comments from package.xml files and fixed license string
  * removed stb_image.h from plotlabserver repository and created automatic download triggered by cmake

## 0.1.0 - 2020-04-15

* Initial Eclipse release
* New ROS nodes
  * adore_if_ros/adore_borderbird_node - simple visualization a simulated vehicle's local view
  * adore_if_ros/adore_feedbackcontroller_node - stabilization of vehicle around reference trajectories: linear feedback controller
  * adore_if_ros/adore_lfbehavior_node - lane-following-only planner (no lane changes)
  * adore_if_ros/adore_objectdetectionmodel_node - a simple (exemplary) sensor model for detection of other traffic participants in simulation
  * adore_if_ros/adore_timer_node - time signal for simulation, pause simulation
  * adore_if_ros/adore_vehiclemodel_node - simulation of a simple vehicle model (linear bicycle model)
  * sumo_if_ros/sumotraffic2ros_node - a node which synchronizes simulation in SUMO and ADORe vehicles in ROS
* New demos 
  * demo003: Simulation of an ADORe automated vehicle, which follows the lanes of an example map
  * demo004: Simulation of three ADORe automated vehicles on a circular track
  * demo005: Simulation of an ADORe automated vehicle and several SUMO vehicles on a circular track
  * demo006: Simulation of an ADORe automated vehicle, which navigates to a goal position by lane following and turning at junctions
* Initial Eclipse release
* libadore/adore/sim: Simulation tools
* libadore/adore/fun: Automated driving control functions, qpOASES based maneuver planner, feed-back control
* libadore/adore/apps, new applications:
    * borderbird.h - simple birds-eye-view plotting application connecting to plotlabserver
    * feedbackcontroller.h - trajectory tracking
    * lane_following_behavior.h - application of qpOASES based maneuver planner to lane following
    * monitor0.h - prototype application for evaluation of important predicates for automatic simulation evaluation
    * objectdetectionmodel.h - application with simple object detection model
    * plainxmlexporter.h - prototype application for exporting of maps to sumo plain xml format
    * test_lc_trajectory_planner.h - test application for lane change maneuver planning with lots of visualizations
    * test_trajectory_planner.h - test application for lane following maneuver planning with lots of visualizations
    * trafficlighsim.h - prototype application for stand-alone traffic light phase simulation
    * vehiclemodel.h - application with vehicle model for use in simulation experiments
  

## 0.0.1 - 2019-11-27

* Test release
* ROS interfaces for adore, ROS message definitions
* New ROS nodes
  * adore_mapprovider_node - loading of OpenDrive tracks for automated vehicle 
  * adore_navigation_node - computation of a shortest route to a given target on loaded track
* New demos
  * demo001: Loading of OpenDrive tracks
  * demo002: Computation of distance functions on track
* libadore/adore/views: Data abstraction for nominal automated driving maneuver planning
* libadore/adore/env: BorderBased realization of LaneFollowingView and LaneChangeView
* libadore/adore/mad: Function library, nonlinear regression for fitting of road coordinate system, interface to qpOASES for dynamic optimization problems
* libadore/adore/if_xodr: OpenDrive file loader
* libadore/adore/apps, new applications:
    * map_provider.h - streaming of static environment information in vehicle vicinity
    * navigation.h - global navigation module / graph search

