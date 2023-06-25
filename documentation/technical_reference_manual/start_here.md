# ADORe Technical Reference Manual

## Table of Contents

- [System and Development](system_and_development.md)
- [Control System](control_system.md)
- [Interfaces](interfaces.md)
- [Doxygen Documentation](doxygen_documentation.md)

The ADORe Technical Reference Manual is divided into several distinct parts.

### 1 System and development
This section will contain details on the ADORe build system, software
components, and software development.

### 2 Control system 
This section will contain details on the ADORe control system used for decision 
making, planning, control and simulation of automated vehicles..

### 3 Interfaces 
This section describes all software interfaces of ADORe.

### 4 Doxygen
Doxygen documentation is provide as part of the ADORe Technical Reference Manual
for the ADORe sources.

## System and development
  1. [Repository structure](system_and_development/repository_structure.md)<!--what is contained in which sub-folder-->
  2. [Sub-folder structure](system_and_development/subfolder_structure.md)<!--file folder with requirements, sub-folder with actual module-->
  3. [Build system](system_and_development/build_system.md)
  4. [ADORe CLI](system_and_development/adore_cli.md)<!--the docker container with a command line interface for development work-->
  5. [Continuous integration and deployment CI/CD](system_and_development/adore_cicd.md)
  6. [Unit tests](system_and_development/unit_tests.md)
  7. [Scenario-based testing](system_and_development/scenario_based_testing.md)
  8. [Static checking tools](system_and_development/static_checking_tools.md)
  9. [Anonymous cloning](system_and_development/anonymous_cloning.md)
  10. [Logging](system_and_development/logging.md)
  11. [Plotting with Plotlab](system_and_development/plotting_plotlab.md)
  12. [Plotting with RViz](system_and_development/plotting_rviz.md)
  13. [ROS-bag replays](system_and_development/rosbag_replay.md)
  13. [Documentation](system_and_development/howto_document.md) <!--how the autogeneration repo/.md to gh-pages/.html works-->
## Interfaces
  1. [ROS](interfaces/ros_integration.md) <!--adore_if_ros and adore_if_ros_msg-->
  2. [Perception](interfaces/perception.md) <!--what is required from perception?-->
  3. [Vehicle Platform](interfaces/vehicle_platform.md) <!--data exchange with base vehicle-->
  4. [V2X messages](interfaces/v2x_messages.md) <!--adore_if_v2x, adore_v2x_sim, v2x_if_ros-->
  5. [SUMO](interfaces/sumo.md) <!--sumo_if_ros-->
  6. [CARLA](interfaces/carla.md) <!--adore_if_carla-->

## Control system
  1. [HD-Map and Navigation](control_system/hd_map_navigation.md)<!--refer to check xodr-->
  2. [Environment model](control_system/environment_model.md)
  3. [Trajectory Planning](control_system/trajectory_planning.md)
  4. [Decision Making](control_system/decision_making.md)
  5. [Stabilization](control_system/stabilization.md)

## Doxygen
[ADORe Doxygen Documentation](generated_doxygen_documentation/index.html)



<!--
4. Simulation
5. Vehicle application
-->
