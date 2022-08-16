title:      ADORe
desc:       Eclipse ADORe is a modular software library and toolkit for decision making, planning, control and simulation of automated vehicles. It is currently developed at DLR (German Aerospace Center), Institute of Transportation Systems.
date:       ${DOC_DATETIME}
version:    ${DOC_VERSION}
template:   document
nav:        Home __1__
percent:    100
authors:    opensource-ts@dlr.de
           
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
********************************************************************************ü
-->


# Automated Driving Open Research (ADORe)
Eclipse ADORe is a modular software library and toolkit for decision making, planning, control and simulation of automated vehicles. 
It is currently developed at DLR (German Aerospace Center), Institute of Transportation Systems.
Some things you will find here:
- Algorithms and data models applied in real automated driving system for motion planning and control
- Mechanisms for safe interaction with other CAVs, infrastructure, traffic management, interactions with human-driven vehicles, bicyclists, pedestrians
- Integration with typical tools and formats such as ROS, [SUMO](https://github.com/eclipse/sumo), CARLA, OpenDrive, Road2Simulation, ITS-G5 V2X (MAPEM, SPATEM, DENM, MCM, SREM)

Here you can see one of our automated test vehicles being operated by ADORe:
[![ADORe example video](https://github.com/DLR-TS/adore_support/blob/master/vivre_flythrough_screenshot2.png?raw=true)](https://youtu.be/tlhPDtr4yxg)

## Quick Start Guide

### Prerequsits 

- Docker v20.10.17 or greater
- docker compose v2.6.0 or greater
- Make
- Storage
  - at least 2.5 GB to clone the repository
  - at least 15 GB to build all necessary docker context

Before you start make sure you have docker, docker compose, and make installed
and configured for your system. To check this run the following commands:
1. check your docker version:
```bash
$ docker --version
Docker version 20.10.17, build 100c701
```
```bash
$ docker compose version
Docker Compose version v2.6.0
```

2. Check that you are a member of the docker group:
```bash
id | grep docker
...,998(docker),...
```

3. Check your storage and be sure you have ~18GB free:
```bash
df -h
```


For help installing and configuring docker follow the official docker documentation (https://docs.docker.com/engine/install/ubuntu/)[https://docs.docker.com/engine/install/ubuntu/]

### Quick Start
1. Clone the repository with submodules:
```bash
git clone --recurse-submodules -j8 git@github.com:eclipse/adore.git
```
2. Build the system:
```bash
cd adore && make
```
3. Run the adore-cli:
```bash
make adore-cli
```
4. Run a test scenario from within the adore-cli context:
```bash
cd adore_if_ros_demos && roslaunch baseline_test.launch
```
5. Run unit tests:
```bash
make test
```
