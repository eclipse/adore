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

# Eclipse ADORe - Automated Driving Open Research
Eclipse ADORe provides a modular software library and toolkit for decision making, planning, control and simulation of automated vehicles.

In scope:
- Algorithms and data models applied in real automated driving system for motion planning and control
- Mechanisms for safe interaction with other CAVs, infrastructure, traffic management, interactions with human-driven vehicles, bicyclists, pedestrians
- Evaluation in context of overall traffic system
- Software quality, reliability and TRL as required for research projects and prototypes

Planned features:
- High-definition road-map representation and loading from OpenDrive file
- Planning modules for smooth "in-lane" driving, (cooperative) lane changes, emergency maneuvers, parking, navigation
- Trajectory tracking module for stabilization of vehicle
- Data models for automation-internal scene understanding, environment models
- Data abstraction views for decoupling of planning algorithms and environment models
- Vehicle model for simulation
- Object detection model (in simulation it replaces the sensor-data fusion pipeline, which is not covered by this project)
- V2X communication model for simulation, (high level, based on look-up tables, no detailed network simulation)
- Interface for co-simulation with Eclipse SUMO: Simulation of traffic and infrastructure around automated vehicle
- Interface to ROS (Robot Operating System)  and possibly other middle-ware interfaces
- Modularity and system independence

Out of scope, not considered:
- Sensor data fusion algorithms for automated driving are currently not covered. The main focus is decision making, not perception. 
- The provided decision making and control algorithms can be evaluated together with sensor/perception models in simulation.
- The project cannot and will not promise creation of highly reliable code, which could be applied in products. The focus is rather on flexible code useful for prototyping and research.
- Non-road-based autonomous systems are not considered.

## Getting started
- [Installation](documentation/installation.md)
- [Executable ROS demos/examples](adore_if_ros_demos)

## Overview
- [libadore](libadore): System-independent library for automated driving functionality.
- [adore_if_ros](adore_if_ros): ROS interface package for ADORe. Uses ROS nodes to run and interconnect applications defined in libadore.
- [adore_if_ros_msg](adore_if_ros_msg): ROS message definitions for data exchange inside an automated vehicle as well as between different automated vehicles in a simulation setup.
- [adore_if_ros_demos](adore_if_ros_demos): ROS launch files for simulation examples.
- [sumo_if_ros](sumo_if_ros): ROS interface package for SUMO. Allows to combine ADORe automated vehicles and [SUMO](http://eclipse.org/sumo) traffic in a ROS-based simulation.
- [plotlabserver](plotlabserver): System-independent plotting.

## Further information
- The current current development state is documented [here](changelog.md)
- The source code and the accompanying material is licensed under the terms of the [EPL v2](LICENSE).
- The source code depends on external software packages. These are listed [here](dependencies.md).
- If you want to get involved, please see [contribution guidelines](CONTRIBUTING.md) and [style guide](documentation/styleguide.md).


## Disclaimer
The views, opinions as well as technical preferences expressed in the source code and its documentation are those of the authors and do not necessarily reflect the official policy or position of their employer.
The technical progress of this open source project is not representative of the DLR-TS closed source development line. 
Some modules cannot be published, due to project funding agreements, commercial licenses and/or dependencies on other closed source modules.
The source code and the accompanying materials are provided without warranty or implication of technical correctness, as governed by [license](LICENSE).
We do not recommend application of the provided source code or material in any kind of safety critical task.

## Contributors
- Daniel Heß
- Stephan Lapoehn
- Thomas Lobig
- Matthias Nichting
- Robert Markowski
- Jan Lauermann
- Reza Deriani
- Jonas Rieck

Former contributors:
- Jörg Belz
- Christian Löper
