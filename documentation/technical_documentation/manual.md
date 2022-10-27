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
# ADORe Manual
1. System and development
  - [Repository structure](system_and_development/repository_structure.md)<!--what is contained in which sub-folder-->
  - [Sub-folder structure](system_and_development/subfolder_structure.md)<!--file folder with requirements, sub-folder with actual module-->
  - [Build process](system_and_development/build_process.md)
  - [ADORe CLI](system_and_development/adore_cli.md)<!--the docker container with a command line interface for development work-->
  - [Continuous integration and deployment CI/CD](system_and_development/adore_cicd.md)
  - [Unit tests](system_and_development/unit_tests.md)
  - [Scenario-based testing](system_and_development/scenario_based_testing.md)
  - [Static checking tools](system_and_development/static_checking_tools.md)
  - [Anonymous cloning](system_and_development/anonymous_cloning.md)
  - [Logging](system_and_development/logging.md)
  - [Plotting with Plotlab](system_and_development/plotting_plotlab.md)
  - [Plotting with RViz](system_and_development/plotting_rviz.md)
  - [ROS-bag replays](system_and_development/rosbag_replay.md)
  - [Documentation](system_and_development/howto_document.md) <!--how the autogeneration repo/.md to gh-pages/.html works-->
2. Interfaces
  - [ROS integration](interfaces/ros_integration.md) <!--adore_if_ros and adore_if_ros_msg-->
  - [Perception](interfaces/perception.md) <!--what is required from perception?-->
  - [Vehicle Platform](interfaces/vehicle_platform.md) <!--data exchange with base vehicle-->
  - [V2X messages](interfaces/v2x_messages.md) <!--adore_if_v2x, adore_v2x_sim, v2x_if_ros-->
  - [SUMO](interfaces/sumo.md) <!--sumo_if_ros-->
  - [CARLA](interfaces/carla.md) <!--adore_if_carla-->
3. Control system
  - [HD-Map and Navigation](control_system/hd_map_navigation.md)<!--refer to check xodr-->
  - [Environment model](control_system/environment_model.md)
  - [Trajectory Planning](control_system/trajectory_planning.md)
  - [Decision Making](control_system/decision_making.md)
  - [Stabilization](control_system/stabilization.md)
<!--
4. Simulation
5. Vehicle application
-->