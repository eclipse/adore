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

## adore_if_ros
A catkin package, which generates ROS nodes for ADORe automated driving
- [src](src) folder: .cpp process wrappers for ROS
- [include/adore_if_ros](include/adore_if_ros) folder: Concrete factory implementations for libadore. Realization of information exchange via ROS.

The most important processes provided by package:

| purpopse                  | ROS node                  | description                                                      |
| ------------------------- | ------------------------- | ---------------------------------------------------------------- |
| planning&control | adore_checkpoint_controller_node | monitors a predefined list of checkpoints and enforces driver interaction before vehicle may continue over a given checkpoint|
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
| simulation | adore_ci_terminator_node | terminates an automatic simulation |
| simulation | adore_objectdetectionmodel_node| simulation module, which transfers information about objects near vehicle from simulation topic to ego vehicle topic|
| simulation | adore_scheduler_node| controls simulation time and advances simulation time according to slowest process|
| simulation | adore_timer_node | constrols simulation time and advances simulation time according to system time (when not paused) |
| simulation | adore_vehiclemodel_node | controls vehicle state in simulation according to scenario initialization and control inputs|

