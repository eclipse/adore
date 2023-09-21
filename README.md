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

# Automated Driving Open Research (ADORe)
Eclipse ADORe is a modular software library and toolkit for decision making, planning, control and simulation of automated vehicles. 
ADORe provides:

- Algorithms and data models applied in real automated driving system for motion planning and control
- Mechanisms for safe interaction with other CAVs, infrastructure, traffic management, interactions with human-driven vehicles, bicyclists, pedestrians
- Integration with typical tools and formats such as ROS, [SUMO](https://github.com/eclipse/sumo), CARLA, OpenDrive, Road2Simulation, ITS-G5 V2X (MAPEM, SPATEM, DENM, MCM, SREM)

The ADORe library (libadore) is written in system-independent c++. For execution and application it is coupled with ROS (adore_if_ros).
Building and execution occurs in docker containers. 

# Overview
An ADORe control system works in concert with a perception stack (not provided) to control an autonomous vehicle platform.
Using V2X radio messages, a list of detected objects and ego vehicle position and velocity, the ADORe control system provides control inputs to a vehicle platform in order to steer it along a given high-definition roadmap to the desired goal location.
![ADORe architectural overview](https://github.com/DLR-TS/adore_support/blob/master/documentation/adore_overview_v03_20221027.png?raw=true)

# Example application
The following video shows an automated vehicle controlled by ADORe in an urban setting in Braunschweig, Germany:
[![ADORe example video](https://github.com/DLR-TS/adore_support/blob/master/adore_vivre_video_preview_20221027.png?raw=true)](https://youtu.be/tlhPDtr4yxg)

# Documentation
In order to get started, it is advised to first check system requirements, follow the installation instruction and then try out the demo scenarios.

- [Github Pages](https://eclipse.github.io/adore/)
- [System requirements](documentation/technical_reference_manual/setup/system_requirements.md)
- [Getting started](documentation/technical_reference_manual/setup/getting_started.md)
- [Executable ROS demo scenarios](https://github.com/DLR-TS/adore_scenarios)
- [Technical Reference Manual](https://eclipse.github.io/adore/mkdocs/about_adore/index.html)
